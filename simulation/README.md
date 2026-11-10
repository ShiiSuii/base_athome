# Simulation of Base@Home

Everything specific to simulation is in this folder. The robot's own code in `src/` runs unchanged: the same mission
executor, voice commands, perception and arm logic. You do not need the robot, the NUC or the Jetson.

## Setup

Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-moveit ros-jazzy-nav2-bringup
git clone -b development https://github.com/ShiiSuii/base_athome.git base_athome_ws
cd base_athome_ws
rosdep install --from-paths src simulation --ignore-src -y
colcon build --symlink-install --packages-up-to base_athome_sim base_athome_skills base_athome_perception base_athome_moveit_config
source install/setup.bash
```

## Run

```bash
# Take out the trash in Gazebo with real Nav2, with its window
GAZEBO=1 NAV2=1 GZ_GUI=1 ATHOME_SIM_BAG=1.5,1.2,0.30 ATHOME_SIM_PERSON=2.6,0.3 ATHOME_SIM_OBJECT_LABEL=handbag \
  simulation/scripts/sim_commander.sh "take out the trash"

# Same mission without a window
GAZEBO=1 NAV2=1 ATHOME_SIM_BAG=1.5,1.2,0.30 ATHOME_SIM_PERSON=2.6,0.3 ATHOME_SIM_OBJECT_LABEL=handbag \
  simulation/scripts/sim_commander.sh "take out the trash"

# Open the commander and send your own requests at http://127.0.0.1:8770
GAZEBO=1 GZ_GUI=1 simulation/scripts/sim_commander.sh

# Pick and Place world (HOME-94): view directly in Gazebo Harmonic
gz sim simulation/base_athome_sim/worlds/casa_pick_place.sdf

# Pick and Place world: launch with robot and bridges in Gazebo
ros2 launch base_athome_sim sim.launch.py gazebo:=true world:=casa_pick_place.sdf

# Pick and Place: commander with real Nav2 on the Pick and Place map
GAZEBO=1 NAV2=1 GZ_GUI=1 ATHOME_SIM_WORLD=casa_pick_place.sdf \
  ATHOME_SIM_MAP=$PWD/simulation/config/casa_pick_place.yaml \
  ATHOME_SIM_SEMANTIC_MAP=$PWD/simulation/base_athome_sim/config/sim_pick_place_semantic_map.yaml \
  simulation/scripts/sim_commander.sh "go to the table"

# GPSR session with scripted speech on /sim/speech, without waiting for the door
COMMANDER_ARGS="-p voice:=true -p tts:=false -p gpsr:=true -p gpsr_wait_door:=false" simulation/scripts/sim_commander.sh

# Lightweight simulator, no physics
simulation/scripts/sim_commander.sh "go to the table"

# Regression scenarios: lightweight simulator, or Gazebo with Nav2
python3 simulation/scripts/sim_escenarios.py
GAZEBO=1 NAV2=1 python3 simulation/scripts/sim_escenarios.py
```

Everything runs in `ROS_DOMAIN_ID=42`. `SIM_DOMAIN` changes it. Never use the robot's domain, 10.

For the shared Receptionist laboratory, use explicit commands rather than the `sim_*` aliases
(those aliases reset the domain to 42):

```bash
export ROS_DOMAIN_ID=77 GZ_PARTITION=receplab DISPLAY=:1 ATHOME_SIM=1
export ATHOME_FACES_DB=/home/athome/sim_lab/receptionist_synthetic_db
ros2 launch base_athome_sim sim.launch.py gazebo:=true gz_gui:=true rviz:=true auto_guest:=true
# In another terminal with the same environment and workspace sourced:
ros2 topic pub --once /sim/guest std_msgs/msg/String '{data: door}'
python3 src/base_athome_perception/scripts/hri_receptionist_challenge.py --sim-nav --no-wait
```

`auto_guest` supplies Robin/iced tea and Morgan/coffee in successive registration requests.
An explicit yes/no confirmation receives `yes` without consuming the next guest's introduction.
It does not seat guests, change their camera image, or simulate acoustic doorbell detection;
use `/sim/guest` and `/sim/speech` to control those interactions. Model weights must already
be installed in the worktree's perception `models/` directory; do not reuse a real facial database.

## Arena: one house, one map, every test

`worlds/arena.sdf` is a 14 x 10 m house (hall with a closed front door, living room, bedroom, kitchen, dining room with a
bar and three tables, laundry with the trash bin) joined by 0.90 m doors. Its map is `simulation/config/arena.yaml` and its
semantic map `base_athome_sim/config/arena_semantic_map.yaml`. The header of the `.sdf` lists rooms, doors and useful
points. `ATHOME_SIM_ARENA=1` selects all three; each test only changes where people, bags and customers are.

```bash
# Navigation through every room with real Nav2
ATHOME_SIM_ARENA=1 GAZEBO=1 NAV2=1 simulation/scripts/sim_commander.sh \
  "go to the kitchen" "go to the bedroom" "go to the door" "go to the dining room" "go to the bar" \
  "go to the laundry" "go to the trash" "go back home"

# Take out the trash: the 12 regression scenarios, placed in the living room of the arena
ATHOME_SIM_ARENA=1 GAZEBO=1 NAV2=1 python3 simulation/scripts/sim_escenarios.py            # all
ATHOME_SIM_ARENA=1 GAZEBO=1 NAV2=1 python3 simulation/scripts/sim_escenarios.py normal     # one

# Nav2 profiles: living-dining door, a person in the way, a person who steps out of the doorway
ATHOME_SIM_ARENA=1 simulation/scripts/nav_eval.py puerta robot

# Restaurant in the dining room: SLAM with no previous map, robot starts at the bar, customers at tables 1 and 3
ROS_DOMAIN_ID=42 python3 simulation/scripts/restaurant_actor.py &      # scripted customers and barman
SIM=1 AUTO=1 ATHOME_SIM_ARENA=1 src/base_athome_skills/scripts/restaurant.sh
```

Regenerate the map after editing the world, then run the tests that check doors, waypoints and reachability:

```bash
python3 simulation/scripts/hacer_mapa.py simulation/base_athome_sim/worlds/arena.sdf simulation/config/arena -4.5 -5.0 16 12
python3 -m pytest simulation/base_athome_sim/test/test_arena_world.py simulation/base_athome_sim/test/test_arena_nav2_reachability.py
```

**STATIC_PASS / UNIT_PASS (2026-11-10), Gazebo pending.** The offline Nav2 test reads the robot's
`nav2_params.yaml` without changing it: radius 0.44 m, resolution 0.05 m, inflation radius 0.59 m,
scaling 4.0, SmacPlanner2D. Nav2 Jazzy 1.3.10's installed `inflation_layer.hpp::computeCost` and
libraries confirm costs 254 at obstacles, 253 at distances <= the inscribed radius, then
`floor(252 * exp(-4 * (distance - inscribed_radius)))`. Distances are between cell centers; the
inflation support is `ceil(0.59 / 0.05) = 12` cells (0.60 m). The default 0.01 m coordinate padding
of the 16-vertex circular footprint gives an inscribed radius of 0.44130138 m. Smac's indexed
`GridCollisionChecker::inCollision` rejects known raw costs >= 253; published OccupancyGrid costs
253/254 become 99/100. The tests use those two scales separately.

All 406 pairs of the 29 semantic waypoints have conservative 8-connected A* paths (no diagonal
corner cutting), with maximum path length 20.27 m and maximum raw cost 243. Each door also passes
a search restricted to its immediate neighborhood, so a remote detour cannot hide a blocked door.
All five openings are 0.90 m, checked against the SDF collision-box edges (also the closed front
door). Sealing each opening breaks its local path, and all six object approach poses are reachable.
In the raster, hall-bedroom and dining-laundry have
0.85 m of free cells and one traversable center; the other three have 0.90 m and two centers.
Their best crossing cost is 243 (published 94). These are cell counts, not measured physical
clearances; no widening or map regeneration was needed for static connectivity.

Restaurant reads its arena defaults from `restaurant.sh`: start/bar `(7.75, 4.4)` facing -90 degrees,
customers `(6.5, 3.5)` and `(8.55, 0.1)`. `approach_candidates` returns 55 and 37 poses with published
cost < 99, inside the dining room and reachable from the bar. The preferred poses `(7.393, 4.143)`
and `(8.883, 1.043)` both have cost 0. Trash and nav_eval points are also checked from home, including
walking destinations and the auxiliary bag; `sin_persona`'s `(-8, -8)` is an explicit absence sentinel.
The trash `apunta_mal` bag moved from `(0.5, 1.3)` to `(0.5, 0.9)` and nav_eval's retreat from
`(5.9, 1.4)` to `(5.8, 1.3)` because both old positions had raw cost 253.

These checks model the static arena map, not dynamic people/bags, online SLAM visibility, perception,
controller execution or physical clearance. No arena mission has yet been run in Gazebo.

The low coffee table in the living room (0.35 m) is below the lidar (0.38 m): it is not in the map and Nav2 does not see
it, as on the real robot. The older worlds and their scenarios still work without `ATHOME_SIM_ARENA`.

## Scene options

| Variable | Meaning |
|---|---|
| `ATHOME_SIM_WORLD=world.sdf` | Gazebo world file (default: `casa_basura.sdf` or `casa_simple.sdf`) |
| `ATHOME_SIM_MAP=map.yaml` | Occupancy map YAML for Nav2 in simulation |
| `ATHOME_SIM_SEMANTIC_MAP=map.yaml` | Semantic map YAML with waypoints, furniture and objects |
| `ATHOME_SIM_ARENA=1` | Use the arena: world, map and semantic map of the whole house |
| `ATHOME_SIM_START=x,y,yaw_deg` | Where the robot appears in Gazebo (default: origin). AMCL starts there too |
| `ATHOME_SIM_BAG=x,y,top` | Bag position and height, in metres |
| `ATHOME_SIM_BAG2=x,y,top` | A second bag, as a distractor |
| `ATHOME_SIM_PERSON=x,y` | Person position |
| `ATHOME_SIM_PERSON_POINTS=0` | The person never points |
| `ATHOME_SIM_POINT_ERR_M=0.15` | How far off the person points |

## What is the same as the robot, and what is not

**Same code as the robot:** mission executor, commander, command parser, `bag_finder`, `bag_vision`, arm manipulation with
MoveIt, the URDF and, with `NAV2=1`, Nav2 with the robot's `nav2_params.yaml`. Its map is generated from the world with
`simulation/scripts/hacer_mapa.py`.

**Simulated in Gazebo:** physics of the bag, lidar, RealSense RGB-D and wrist camera.

**Simplified:**
- Without `NAV2=1`, navigation is a stand-in action server.
- The base moves by velocity with gravity off: wheels are not simulated.
- Bags are detected by colour, not with YOLO-World. The bag is a magenta box, not a deformable bag.
- People's body points come from the simulator, not from YOLO pose.
- Speech is typed through `/sim/speech`, and the arm, head and lift firmware are stand-ins.
- A grasp succeeds when the gripper closes within 6 cm of the knot point.

**A result here does not show the robot is safe or that a real grasp works.**
