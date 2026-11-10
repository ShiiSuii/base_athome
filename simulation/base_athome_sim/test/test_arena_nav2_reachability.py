"""Offline Nav2 Jazzy (1.3.10) costmap and A* checks; never starts ROS/Gazebo.

Local evidence: /opt/ros/jazzy/include/nav2_costmap_2d/nav2_costmap_2d/
inflation_layer.hpp::computeCost (<= inscribed, unsigned-char truncation).
The installed libnav2_smac_planner.so Node2D::isNodeValid calls the indexed
GridCollisionChecker::inCollision: known costs > 252 collide (objdump).
Inflation uses cellDistance = ceil(radius/resolution), including its last ring.
Costmap2DROS makes a 16-vertex circle and pads each coordinate by sign0 * 0.01
(default footprint_padding), then LayeredCostmap uses its minimum edge distance.
These defaults/functions were checked in libnav2_costmap_2d_core.so, without
constructing nodes. OccupancyGrid conversion is Costmap2DPublisher's lookup.
This proves static grid connectivity, not controller/smoother/SLAM behavior.
"""

import heapq
import importlib.util
import itertools
import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

# Always exercise this worktree, even when the caller sources another install.
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/base_athome_skills"))
from base_athome_skills.mission.restaurant_session import approach_candidates
from base_athome_skills.mission.places import point_in_polygon

SIM = ROOT / "simulation"
SEMANTIC = yaml.safe_load((SIM / "base_athome_sim/config/arena_semantic_map.yaml").read_text())
DOORS = [name for name, door in SEMANTIC["doors"].items() if not door.get("closed")]
WAYPOINT_PAIRS = list(itertools.combinations(SEMANTIC["waypoints"], 2))


def load_module(filename, name):
    spec = importlib.util.spec_from_file_location(name, SIM / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)  # functions/constants only; never main()
    return module


def inscribed_radius(radius, padding):
    """Minimum origin-to-edge distance of Nav2's padded circular footprint."""
    points = []
    # Preserve signs of floating point sin/cos, as makeFootprintFromRadius does.
    for i in range(16):
        x, y = radius * math.cos(i * math.tau / 16), radius * math.sin(i * math.tau / 16)
        points.append((x + (padding if x > 0 else -padding if x < 0 else 0),
                       y + (padding if y > 0 else -padding if y < 0 else 0)))
    distances = []
    for a, b in zip(points, points[1:] + points[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0., min(1., -(a[0] * dx + a[1] * dy) / (dx * dx + dy * dy)))
        distances.append(math.hypot(a[0] + t * dx, a[1] + t * dy))
    return min(distances)


def inflate(occupied, resolution, inscribed, inflation, scaling):
    """Max of cached InflationLayer costs, using Euclidean cell-center distances."""
    costs = np.zeros(occupied.shape, dtype=np.uint8)
    h, w = occupied.shape
    n = math.ceil(inflation / resolution)
    for dr in range(-n, n + 1):
        for dc in range(-n, n + 1):
            distance = math.hypot(dr, dc)
            if distance > n:
                continue
            meters = distance * resolution
            cost = (254 if distance == 0 else 253 if meters <= inscribed
                    else int(252 * math.exp(-scaling * (meters - inscribed))))
            src = occupied[max(0, -dr):min(h, h - dr), max(0, -dc):min(w, w - dc)]
            dst = costs[max(0, dr):min(h, h + dr), max(0, dc):min(w, w + dc)]
            np.maximum(dst, np.where(src, cost, 0).astype(np.uint8), out=dst)
    return costs


def occupancy_values(costs):
    """Raw 0..255 Nav2 -> published -1..100; never confuse 253 with 99."""
    lut = np.array([0] + [1 + 97 * (i - 1) // 251 for i in range(1, 253)]
                   + [99, 100, -1], dtype=np.int16)
    return lut[costs]


class ArenaCostmap:
    def __init__(self):
        params = yaml.safe_load((ROOT / "src/base_athome_navigation/config/nav2_params.yaml").read_text())
        self.params = params
        p = params["global_costmap"]["global_costmap"]["ros__parameters"]
        self.res = p["resolution"]
        self.radius = p["robot_radius"]
        # ParameterValue(float) is stored as float in Costmap2DROS.
        padding = float(np.float32(p.get("footprint_padding", 0.01)))
        self.inscribed = inscribed_radius(self.radius, padding)
        info = yaml.safe_load((SIM / "config/arena.yaml").read_text())
        assert info["resolution"] == self.res and info["origin"][2] == 0
        assert info["negate"] == 0 and info["mode"] == "trinary"
        self.origin = info["origin"][:2]
        with (SIM / "config" / info["image"]).open("rb") as f:
            assert f.readline().strip() == b"P5"
            w, h = map(int, f.readline().split())
            assert int(f.readline()) == 255
            image = np.frombuffer(f.read(), np.uint8).reshape(h, w)[::-1]
        # MapServer trinary conversion; assert this arena has no unknown pixels.
        probability = (255 - image.astype(float)) / 255
        self.occupied = probability > info["occupied_thresh"]
        assert np.all(self.occupied | (probability < info["free_thresh"]))
        inf = p["inflation_layer"]
        self.costs = inflate(self.occupied, self.res, self.inscribed,
                             inf["inflation_radius"], inf["cost_scaling_factor"])
        self.path_metrics = []
        values = occupancy_values(self.costs)
        self.grid = NS(info=NS(width=w, height=h, resolution=self.res,
                              origin=NS(position=NS(x=self.origin[0], y=self.origin[1]),
                                        orientation=NS(x=0., y=0., z=0., w=1.))),
                       data=values.ravel().tolist())

    def cell(self, xy):
        return (math.floor((xy[1] - self.origin[1]) / self.res),
                math.floor((xy[0] - self.origin[0]) / self.res))

    def cost(self, xy):
        r, c = self.cell(xy)
        assert 0 <= r < self.costs.shape[0] and 0 <= c < self.costs.shape[1], xy
        return int(self.costs[r, c])

    def astar(self, start, goal, costs=None):
        """8-connected, cost-weighted A*. No goal tolerance or blocked endpoints.

        Disallow corner cutting as an additional conservative constraint.
        This is a connectivity witness, not a reimplementation of Smac smoothing.
        """
        costs = self.costs if costs is None else costs
        start, goal = self.cell(start), self.cell(goal)
        h, w = costs.shape
        costs = costs.tolist()
        for r, c in (start, goal):
            if not (0 <= r < h and 0 <= c < w) or costs[r][c] >= 253:
                return []
        queue, best, parent = [(0., 0., start)], {start: 0.}, {}
        while queue:
            _, g, here = heapq.heappop(queue)
            if g > best[here]:
                continue
            if here == goal:
                path = [here]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                return path[::-1]
            r, c = here
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                rr, cc = r + dr, c + dc
                if not (0 <= rr < h and 0 <= cc < w) or costs[rr][cc] >= 253:
                    continue
                if dr and dc and (costs[r][cc] >= 253 or costs[rr][c] >= 253):
                    continue
                ng = g + math.hypot(dr, dc) * (1 + costs[rr][cc] / 252.)
                neighbor = (rr, cc)
                if ng < best.get(neighbor, math.inf):
                    best[neighbor], parent[neighbor] = ng, here
                    heuristic = math.hypot(goal[0] - rr, goal[1] - cc)
                    heapq.heappush(queue, (ng + heuristic, ng, neighbor))
        return []


@pytest.fixture(scope="module")
def arena():
    costmap = ArenaCostmap()
    yield costmap
    if costmap.path_metrics:
        print(f"A*: {len(costmap.path_metrics)} waypoint pairs; "
              f"max path length={max(p[0] for p in costmap.path_metrics):.2f} m; "
              f"max raw cost={max(p[1] for p in costmap.path_metrics)}")


def xy(p):
    return p["x"], p["y"]


def test_effective_nav2_parameters(arena):
    for key in ("local_costmap", "global_costmap"):
        p = arena.params[key][key]["ros__parameters"]
        assert not p.get("footprint")
        assert (p["robot_radius"], p["resolution"]) == (0.44, 0.05)
        assert (p["inflation_layer"]["inflation_radius"], p["inflation_layer"]["cost_scaling_factor"]) == (0.59, 4.0)
    planner = arena.params["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert planner["downsample_costmap"] is False
    assert arena.inscribed == pytest.approx(0.44130138, abs=1e-8)


def test_inflation_boundary_and_published_costs():
    occupied = np.zeros((31, 31), bool)
    occupied[15, 15] = True
    costs = inflate(occupied, 0.05, 0.44, 0.59, 4.)
    # Inclusive inscribed boundary, diagonal Euclidean distance, truncated costs,
    # ceil(0.59/0.05)=12 means the 0.60 m ring is still inflated.
    assert [int(costs[15, 15 + i]) for i in (0, 8, 9, 11, 12, 13)] == [254, 253, 242, 162, 132, 0]
    assert costs[21, 21] == 253 and costs[22, 22] == 202
    assert occupancy_values(np.array([0, 1, 252, 253, 254, 255])).tolist() == [0, 1, 98, 99, 100, -1]
    boundary = inflate(occupied, 0.05, 0.45, 0.59, 4.)
    assert boundary[15, 24] == 253


@pytest.mark.parametrize("a,b", WAYPOINT_PAIRS)
def test_all_semantic_waypoint_pairs_have_astar_path(arena, a, b):
    path = arena.astar(xy(SEMANTIC["waypoints"][a]), xy(SEMANTIC["waypoints"][b]))
    assert path, f"{a} -> {b}: no path with Nav2 costs < 253"
    assert max(int(arena.costs[cell]) for cell in path) < 253
    length = sum(math.hypot(q[0] - p[0], q[1] - p[1]) * arena.res for p, q in zip(path, path[1:]))
    arena.path_metrics.append((length, max(int(arena.costs[cell]) for cell in path)))


@pytest.mark.parametrize("name", DOORS)
def test_each_door_can_be_crossed_locally(arena, name):
    door = SEMANTIC["doors"][name]
    x, y = xy(door)
    # Restrict search to doorway; a distant alternative route cannot hide failure.
    along_x = door["axis"] == "y"
    def side_point(sign):
        # Kitchen table is immediately beyond its door: choose a free arrival
        # on that side instead of mistaking a table collision for a closed door.
        for offset in sorted(range(-14, 15), key=abs):
            point = ((x + sign * 0.65, y + offset * arena.res) if along_x
                     else (x + offset * arena.res, y + sign * 0.65))
            if arena.cost(point) < 253:
                return point
        pytest.fail(f"{name}: no free approach on side {sign}")

    start, goal = side_point(-1), side_point(1)
    local = np.full_like(arena.costs, 254)
    r0, c0 = arena.cell((x - 0.8, y - 0.8))
    r1, c1 = arena.cell((x + 0.8, y + 0.8))
    local[r0:r1 + 1, c0:c1 + 1] = arena.costs[r0:r1 + 1, c0:c1 + 1]
    path = arena.astar(start, goal, local)
    assert path, f"{name}: doorway blocked by Nav2 inflation"
    r, c = arena.cell((x, y))
    # Negative control: sealing the doorway must break this local crossing.
    sealed = local.copy()
    if along_x:
        sealed[:, c] = 254
    else:
        sealed[r, :] = 254
    assert not arena.astar(start, goal, sealed), f"{name}: search bypassed the doorway"
    lo = hi = c if door["axis"] == "x" else r
    line = arena.occupied[r, :] if door["axis"] == "x" else arena.occupied[:, c]
    while lo > 0 and not line[lo - 1]:
        lo -= 1
    while hi + 1 < len(line) and not line[hi + 1]:
        hi += 1
    cross = arena.costs[r, lo:hi + 1] if door["axis"] == "x" else arena.costs[lo:hi + 1, c]
    print(f"{name}: SDF={door['width_m']:.2f} m; PGM={(hi-lo+1)*arena.res:.2f} m; "
          f"traversable centers={np.count_nonzero(cross < 253)}; min raw cost={int(cross.min())}; "
          f"local A*={len(path)} cells")


@pytest.mark.parametrize("name", SEMANTIC["objects"])
def test_object_approaches_reachable_from_home(arena, name):
    point = xy(SEMANTIC["objects"][name]["approach"])
    assert arena.cost(point) < 253, (name, point, arena.cost(point))
    assert arena.astar((0, 0), point), (name, point)


def shell_arena_default(name):
    text = (ROOT / "src/base_athome_skills/scripts/restaurant.sh").read_text()
    block = text.split('if [ "${ATHOME_SIM_ARENA:-0}" = 1 ]; then', 1)[1].split("  else", 1)[0]
    return re.search(r"\$\{" + name + r":-([^}]+)\}", block).group(1)


def test_restaurant_start_bar_and_customer_approaches(arena):
    start = tuple(map(float, shell_arena_default("ATHOME_SIM_START").split(",")))
    bar = xy(SEMANTIC["waypoints"]["bar"])
    assert start[:2] == bar and start[2] == -90
    assert arena.cost(start) < 253
    assert arena.astar((0, 0), bar)
    for entry in shell_arena_default("ATHOME_SIM_CUSTOMERS").split(";"):
        customer = tuple(map(float, entry.split(",")[:2]))
        assert point_in_polygon(*customer, SEMANTIC["zones"]["dining_room"]["polygon"])
        candidates = approach_candidates(customer, (start[0], start[1], math.radians(start[2])), arena.grid)
        assert len(candidates) > 1, "fallback is not evidence of a valid approach"
        for pose in candidates:
            assert arena.cost(pose) < 253, (customer, pose, arena.cost(pose))
            r, c = arena.cell(pose)
            assert arena.grid.data[r * arena.grid.info.width + c] < 99
            assert point_in_polygon(*pose[:2], SEMANTIC["zones"]["dining_room"]["polygon"])
            assert arena.astar(bar, pose), (customer, pose)
        assert arena.astar(bar, candidates[0]), (customer, candidates[0])
        assert arena.astar(candidates[0], bar)
        print(f"Restaurant {customer}: {len(candidates)} candidates; "
              f"first={tuple(round(v, 3) for v in candidates[0][:2])}; "
              f"raw cost={arena.cost(candidates[0])}; "
              f"published={arena.grid.data[arena.cell(candidates[0])[0] * arena.grid.info.width + arena.cell(candidates[0])[1]]}")


def test_trash_person_bags_and_walks_reachable_from_home(arena):
    module = load_module("sim_escenarios.py", "arena_reachability_scenarios")
    for scenario in module.arena_scenarios():
        for key in ("ATHOME_SIM_PERSON", "ATHOME_SIM_BAG", "ATHOME_SIM_BAG2", "ATHOME_SIM_PERSON_WALK"):
            for entry in scenario.env.get(key, "").split(";"):
                if not entry:
                    continue
                point = tuple(map(float, entry.split(",")[:2]))
                if scenario.name == "sin_persona" and key == "ATHOME_SIM_PERSON":
                    assert point == (-8., -8.)  # explicit absence sentinel, outside map
                    continue
                assert arena.cost(point) < 253, (scenario.name, key, point, arena.cost(point))
                assert arena.astar((0, 0), point), (scenario.name, key, point)


def test_nav_eval_people_bag_walks_and_targets_reachable(arena):
    module = load_module("nav_eval.py", "arena_reachability_nav_eval")
    for name, scenario in module.ARENA_SCENARIOS.items():
        points = [scenario["person"], *scenario.get("walk", "").split(";")]
        for entry in points:
            if entry:
                point = tuple(map(float, entry.split(",")[:2]))
                assert arena.cost(point) < 253, (name, entry, arena.cost(point))
                assert arena.astar((0, 0), point), (name, entry)
        target = scenario["command"].removeprefix("go to the ").replace(" ", "_")
        assert arena.astar((0, 0), xy(SEMANTIC["waypoints"][target]))
    # Extract the arena bag from run() without executing it or creating a Recorder.
    text = (SIM / "scripts/nav_eval.py").read_text().split("def run(", 1)[1].split("if ARENA else", 1)[0]
    bag = tuple(map(float, re.search(r'ATHOME_SIM_BAG="([^"]+)"', text).group(1).split(",")[:2]))
    assert arena.cost(bag) < 253 and arena.astar((0, 0), bag), (bag, arena.cost(bag))
