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
python3 -m pytest simulation/base_athome_sim/test/test_arena_world.py
```

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
