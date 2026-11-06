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

# Lightweight simulator, no physics
simulation/scripts/sim_commander.sh "go to the table"

# Regression scenarios: lightweight simulator, or Gazebo with Nav2
python3 simulation/scripts/sim_escenarios.py
GAZEBO=1 NAV2=1 python3 simulation/scripts/sim_escenarios.py
```

Everything runs in `ROS_DOMAIN_ID=42`. `SIM_DOMAIN` changes it. Never use the robot's domain, 10.

## Scene options

| Variable | Meaning |
|---|---|
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
