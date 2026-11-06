# Base@Home (`base_athome`)

**Base@Home** is the domestic service robot developed by **UruBots** (Uruguay) for the **RoboCup@Home Open Platform League (OPL)**. It is a mobile manipulation and human-robot interaction (HRI) platform running entirely on **ROS 2 Jazzy Jalisco** (Ubuntu 24.04 LTS) across two onboard computers connected over Gigabit Ethernet.

---

## 1. Robot & Team

- **Team:** UruBots (Uruguay) — competing in RoboCup@Home Open Platform League (2026.2 RuleBook reference; 2027 qualification cycle).
- **Platform:** Custom differential-drive service robot with a motorized vertical lift column, a 6-DOF anthropomorphic manipulator with parallel gripper and eye-in-hand RGB-D camera, a 2-DOF pan-tilt head with RGB-D perception and directional audio, and an animated facial display.
- **Design philosophy:** Distributed ROS 2 architecture where each sensor and actuator driver runs on the physical host attached to its hardware peripheral, communicating via deterministic CycloneDDS unicast over a wired Gigabit link (no team Wi-Fi hotspot, compliant with RoboCup@Home §3.2.8). All actuator controllers enforce hardware-off startup defaults (`auto_enable=False`, `auto_torque=false`) and require fresh hardware feedback before accepting motion commands.

---

## 2. Hardware Overview

| Subsystem | Component | Interface & Host | Notes |
|---|---|---|---|
| **Main Compute (NUC)** | CHUWI LarkBox S (Intel Core i3-1220P, 10C/12T, 15 GiB RAM, NVMe) | Ubuntu 24.04.3 LTS · ROS 2 Jazzy | Runs Nav2, AMCL, head camera driver, HRI/receptionist state machines, MoveIt 2, mission execution, and local UI/TTS |
| **Edge GPU Compute (Jetson)** | NVIDIA Jetson Orin Nano Dev Kit Super (8 GB, 6× Cortex-A78AE, Ampere GPU, 25W mode) | Ubuntu 24.04.4 LTS · L4T R39.2.1 (JetPack 7.2.1) · CUDA 13.2 · TensorRT 10.16.2 · ROS 2 Jazzy | Runs 50 Hz arm control, pan-tilt driver, wrist camera streaming, TensorRT YOLO inference, and Whisper ASR (`jetson-head.service`) |
| **Mobile Base** | Differential drive with 2× 6.5" BLDC hoverboard hub motors (Hall odometry, 90 ticks/rev, `wheel_radius=0.0853 m`, `track_width=0.5620 m`) | STM32F103 FOC board (`VLT_MODE`) over USB-TTL UART (`115200` baud) on NUC | 50 Hz `/odom` and TF `odom -> base_footprint`; 0.5 s `cmd_vel` watchdog |
| **2D LiDAR** | Hokuyo UST-05LX (270° FOV, 0.06–5.0 m range, 40 Hz) | Dedicated Ethernet link on NUC (`urg_node`) | Mounted at `z = 0.32 m` above floor (beam plane at `0.38 m`) |
| **Torso Lift Column** | Lead-screw vertical elevator driven by NEMA 23 stepper + ESP32-S3 controller | USB-Serial (`/dev/ttyACM0`) on NUC | Prismatic `torso_lift_joint` (model range `0.0` down to `-0.75 m`) |
| **Manipulator** | AgileX Piper 6-DOF robot arm (1.5 kg payload, ~626 mm reach) + parallel gripper | `gs_usb` CAN bus at 1 Mbit/s on Jetson (`piper_sdk` 0.6.2) | Side-mounted on torso cube; 50 Hz control loop + `FollowJointTrajectory` action server |
| **Wrist Camera (Eye-in-Hand)** | Orbbec DaBai DC1 structured-light RGB-D camera (effective depth 0.3–3.0 m) | USB on Jetson (`stream_orbbec.py`) | Mounted above gripper base (`arm_camera_optical_frame`); feeds GPU TensorRT detector |
| **Head Camera** | Intel RealSense D435 RGB-D camera (`640x480 @ 15 Hz` color + aligned depth) | USB 3 on NUC (`realsense2_camera`) | Used for face recognition, human pose estimation, seat occupancy, and 3D bag localization |
| **Pan-Tilt Neck** | 2× Dynamixel XL430-W250 smart servos (Protocol 2.0, 1 Mbaud, extended position mode) | Robotis U2D2 (`/dev/ttyU2D2`) on Jetson | Drives `pan_joint` and `tilt_joint` at 20 Hz |
| **Audio & HRI Display** | RØDE VideoMic GO II directional USB shotgun microphone + HDMI facial display & TTS speaker | Mic on Jetson USB; HDMI display/audio on NUC | Request-driven ASR (`whisper_node` + Silero VAD), animated face (`/robot/emotion`), and fallback local web touch screen (`:8090`) |

---

## 3. System Architecture

```text
+---------------------------------------------------------------------------------------------------+
|                                   Base@Home Distributed System                                    |
|                                                                                                   |
|   NUC (Ubuntu 24.04 · ROS 2 Jazzy)                    Jetson Orin Nano (L4T R39.2.1 · Jazzy)      |
|  +------------------------------------------------+  +-----------------------------------------+  |
|  | [urg_node] <--- Ethernet --- Hokuyo UST-05LX   |  | [piper_arm_controller] <-- CAN -- Piper |  |
|  |      | (/scan 40 Hz)                           |  |   (/arm/joint_states, /arm/status,      |  |
|  |      v                                         |  |    /arm/follow_joint_trajectory)        |  |
|  | [Nav2: map_server + AMCL + SmacPlanner2D +     |  |                                         |  |
|  |        RotationShim / RegulatedPurePursuit]    |  | [pan_tilt_node] <-- U2D2 -- XL430 (x2)  |  |
|  |      | (/cmd_vel)             ^ (/odom, TF)    |  |   (/pan_tilt/joint_states, /connected)  |  |
|  |      v                        |                |  |                                         |  |
|  | [hoverboard_driver] <-- UART -+- STM32 FOC     |  | [stream_orbbec] <-- USB -- DaBai DC1    |  |
|  |                                                |  |      | (/arm_camera/image_raw ~20-30Hz) |  |
|  | [realsense2_camera] <-- USB3 -- RealSense D435 |  |      v                                  |  |
|  |      | (RGB + aligned depth 15 Hz)             |  | [yolo_trt_node] (TensorRT 10.16 FP16)   |  |
|  |      v                                         |  |   (/arm_camera/detections ~30 Hz)       |  |
|  | [Perception & HRI: YuNet/SFace, YOLOv8-pose,   |  |                                         |  |
|  |  bag_vision, hri_receptionist, manipulacion]   |  | [whisper_node] <-- USB -- RODE VideoMic |  |
|  |      ^                                         |  |   (Silero VAD + faster-whisper ->       |  |
|  |      | (NavigateToPose, MoveIt 2, /voice/*)    |  |    /voice/result, /voice/transcription) |  |
|  |      v                                         |  |                                         |  |
|  | [Skills & Missions: door_open_detector,        |  | [quest_teleop_node] <-- ADB USB -- VR   |  |
|  |  mission_executor, commander (:8770)]          |  +-----------------------------------------+  |
|  |                                                |                       ^                       |
|  | [torso_state / stepper] <-- USB -- ESP32-S3    |                       |                       |
|  | [animated_face + TTS + guest web UI (:8090)]   |                       |                       |
|  +------------------------------------------------+                       |                       |
|                         ^                                                 |                       |
|                         +--- Direct Gigabit Ethernet · CycloneDDS Unicast +                       |
|                              (ROS_DOMAIN_ID=10 on robot · ROS_DOMAIN_ID=42 in sim)                |
+---------------------------------------------------------------------------------------------------+
```

For full technical details on hardware interfaces, TF tree (`map -> odom -> base_footprint -> base_link -> ...`), calibrations, and safety guards, see **[`docs/ROBOT.md`](docs/ROBOT.md)**.

---

## 4. Packages in `src/`

Each package under `src/` is built with `colcon` on ROS 2 Jazzy:

- **[`base_athome_bringup`](src/base_athome_bringup)**: Hardware launch files for the mobile base (`base.launch.py`) and head camera (`realsense_nuc.launch.py`), plus the `stepper_controller` and read-only/commandable `torso_state` nodes for the ESP32-S3 elevator.
- **[`base_athome_mapping`](src/base_athome_mapping)**: 2D and 3D SLAM launch configurations (`slam_toolbox.launch.py`, `slam_2d_mapping.launch.py`, `slam_3d_mapping.launch.py`, `rtabmap_realsense.launch.py`).
- **[`base_athome_moveit_config`](src/base_athome_moveit_config)**: MoveIt 2 configuration for the full robot URDF (`arm` and `gripper` planning groups, TRAC-IK kinematics, OMPL planners, and sampled Allowed Collision Matrix).
- **[`base_athome_navigation`](src/base_athome_navigation)**: Autonomous navigation stack (`nav2_bringup.launch.py`, AMCL, `SmacPlanner2D`, `RotationShimController` + `RegulatedPurePursuit`), PS4 teleop, waypoint navigation, and the offline semantic map & zone editor (`map_zone_editor.py`, `semantic_objects.py`).
- **[`base_athome_perception`](src/base_athome_perception)**: Vision, audio client, HRI Receptionist state machine, YuNet/SFace face recognition, YOLOv8 human pose & seat detection, unified 3D garbage/bag perception (`bag_vision.py`, `bag_finder.py`), MoveIt manipulation bridge (`moveit_bridge.py`, `arm_manipulation.py`), and animated face/guest UI.
- **[`base_athome_sim`](src/base_athome_sim)**: Lightweight kinematic/sensor simulator (`sim_robot`) and optional Gazebo Harmonic (`gz sim`) bridge that mock the Jetson nodes, cameras, ASR, and Nav2 under identical ROS 2 interfaces in `ROS_DOMAIN_ID=42`.
- **[`base_athome_skills`](src/base_athome_skills)**: Competition skills and deterministic mission layer, including the LiDAR start-signal `door_open_detector` (RuleBook §3.6.8), validated `mission_executor`, and natural-language `commander` node with web UI (`:8770`). See [`src/base_athome_skills/README.md`](src/base_athome_skills/README.md).
- **[`hoverboard_base`](src/hoverboard_base)**: Serial FOC protocol driver (`hoverboard_driver.py` with Hall odometry and `cmd_vel` watchdog) and the modular robot Xacro/URDF description (`urdf/robot.urdf.xacro`, 30 joints / 11 actuated).
- **[`piper_description`](src/piper_description)**: URDF meshes and standalone visualization launch files for the AgileX Piper 6-DOF robotic arm and parallel gripper.
- **[`piper_msgs`](src/piper_msgs)**: Custom ROS 2 message and service definitions (`PiperStatusMsg`, `PosCmd`, `Enable`, `GoZero`, `Gripper`) for the AgileX Piper arm.
- **[`urg_node`](src/urg_node)**: ROS 2 driver package for the Hokuyo UST-05LX Ethernet 2D LiDAR publishing `/scan`.

Additional repository directories outside `src/`:
- **[`jetson_head/`](jetson_head)**: Source packages (`audio_asr`, `pan_tilt_controller`, `robot_bringup_jetson`) and scripts (`piper_arm_controller.py`, `stream_orbbec.py`, `yolo_trt_node.py`, `quest_teleop_node.py`, `deploy_to_jetson.sh`) deployed to the Jetson Orin Nano (excluded from NUC `colcon` builds via `COLCON_IGNORE`).
- **[`tools/gpsr_parser/`](tools/gpsr_parser/README.md)**: Deterministic offline parser converting GPSR natural-language commands into structured JSON mission plans.
- **[`tools/urdf_editor/`](tools/urdf_editor/README.md)**: Local web-based Xacro/URDF parameter preview and 3D inspection tool (`http://127.0.0.1:8765`).
- **[`firmware/`](firmware/README.md)**: Embedded C/C++ firmware and wiring documentation ([`firmware/docs/PINOUTS_AND_WIRING.md`](firmware/docs/PINOUTS_AND_WIRING.md)) for the STM32F103 hoverboard controller and ESP32-S3 NEMA 23 lift controller.

---

## 5. Installation (ROS 2 Jazzy on Ubuntu 24.04)

### 5.1 System Dependencies (NUC / Development Workstation)

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-realsense2-camera \
  ros-jazzy-urg-node \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-xacro \
  ros-jazzy-moveit \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  liburdfdom-tools \
  python3-colcon-common-extensions \
  python3-pytest \
  python3-venv \
  python3-opencv \
  python3-yaml
```

### 5.2 Python Vision Environment

Perception and HRI scripts use a virtual environment with access to system ROS 2 packages:

```bash
python3 -m venv ~/ros2_vision_venv --system-site-packages
~/ros2_vision_venv/bin/pip install ultralytics opencv-python pyyaml
```

### 5.3 Build the Workspace

```bash
git clone https://github.com/ShiiSuii/base_athome.git ~/base_athome_ws
cd ~/base_athome_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

*(Note: Several perception launch files and helper aliases expect the workspace at `~/base_athome_ws` and the Python vision virtual environment at `~/ros2_vision_venv`.)*

### 5.4 Verify URDF and Unit Tests

Run the offline test suites in an isolated DDS domain (`ROS_DOMAIN_ID=77`):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Validate Xacro expansion and URDF tree (30 joints)
xacro src/hoverboard_base/urdf/robot.urdf.xacro > /tmp/base_athome.urdf
check_urdf /tmp/base_athome.urdf

# Run unit tests across ROS 2 packages and Jetson scripts
ROS_DOMAIN_ID=77 python3 -m pytest -q \
  src/base_athome_perception/test \
  src/base_athome_sim/test \
  src/base_athome_navigation/test \
  src/base_athome_bringup/test/test_torso_state.py \
  jetson_head/scripts/tests \
  jetson_head/src/pan_tilt_controller/test \
  jetson_head/src/audio_asr/test/test_asr_core.py

# Skills & mission executor test suite
ROS_DOMAIN_ID=77 env -u PYTHONPATH python3 -m pytest -q src/base_athome_skills/test

# Standalone GPSR parser test suite
(cd tools/gpsr_parser && env -u PYTHONPATH python3 -m pytest -q tests)
```

### 5.5 Jetson Orin Nano Setup (Optional — Hardware Only)

The Jetson runs Ubuntu 24.04 (L4T R39.2.1 / JetPack 7.2.1) and ROS 2 Jazzy. Its code in `jetson_head/` is deployed from the NUC via `jetson_head/scripts/deploy_to_jetson.sh` (runs in dry-run mode by default; pass `--apply` to sync) and managed by systemd (`jetson-head.service`). Initial environment provisioning is scripted in `jetson_head/scripts/bootstrap_jazzy.sh`. For microphone and ASR diagnostics, see [`docs/MICROFONO_ASR.md`](docs/MICROFONO_ASR.md).

---

## 6. Running the Simulation (Without Hardware)

`base_athome_sim` replaces the Jetson nodes (Piper arm, pan-tilt neck, Whisper ASR, wrist camera), the RealSense camera, and Nav2 with simulated counterparts using identical ROS 2 topics, services, and actions in **`ROS_DOMAIN_ID=42`** (`ATHOME_SIM=1`).

### 6.1 Lightweight Kinematic Simulation + RViz2

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ATHOME_SIM=1

# Launch simulated robot + RViz2 (no physics engine, minimal CPU)
ros2 launch base_athome_sim sim.launch.py

# Or launch with automatic simulated guest speech responses:
ros2 launch base_athome_sim sim.launch.py auto_guest:=true
```

To run with **Gazebo Harmonic** (`gz sim`) physics and sensor bridge instead:

```bash
export ROS_DOMAIN_ID=42 ATHOME_SIM=1
ros2 launch base_athome_sim sim.launch.py gazebo:=true
```

### 6.2 Interacting with the Simulated Robot (Second Terminal)

In another terminal configured for domain `42`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ATHOME_SIM=1

# Run the HRI Receptionist state machine against the simulator
python3 src/base_athome_perception/scripts/hri_receptionist_challenge.py --sim-nav --max-guests 1

# Or trigger simulated guest appearance and speech manually:
ros2 topic pub --once /sim/guest std_msgs/msg/String "{data: 'door'}"
ros2 topic pub --once /sim/speech std_msgs/msg/String "{data: 'My name is Robin and my favorite drink is iced tea'}"
```

### 6.3 End-to-End Simulated Missions & Natural-Language Commander

Self-contained scripts launch `base_athome_sim`, `mission_executor`, and `commander` in `ROS_DOMAIN_ID=42` and clean up all spawned processes on exit:

```bash
# Interactive web Commander at http://127.0.0.1:8770 (add RVIZ=1 to open RViz2)
src/base_athome_skills/scripts/sim_commander.sh

# Automated natural-language compound command execution in simulation
src/base_athome_skills/scripts/sim_commander.sh "anda a la mesa, busca una botella y llevala a la cocina"

# Execute a YAML mission plan in simulation
src/base_athome_skills/scripts/sim_mission.sh \
  src/base_athome_skills/missions/demo_fetch_bottle.yaml \
  src/base_athome_sim/config/sim_semantic_map.yaml

# Visual closed-loop simulation demo of the "take out the trash" task (with RViz2)
src/base_athome_skills/scripts/sim_basura.sh
```

> **Note:** Simulation validates state machines, kinematics, TF, and ROS 2 interfaces (`SIM_PASS`), but does **not** demonstrate physical safety, real floor navigation obstacle avoidance, real audio acoustics, or physical grasping of deformable objects.

---

## 7. Capability Status

Current verification status separated strictly by evidence level (**Verified on Hardware**, **Verified in Simulation**, and **Code / Unit Tests Only**):

| Capability | Verified on Hardware | Verified in Simulation (`SIM_PASS`) | Code / Unit Tests Only (`UNIT_PASS` / `CODIGO`) |
|---|---|---|---|
| **Compute & Distributed DDS** | NUC (Ubuntu 24.04, Jazzy) + Jetson Orin Nano (L4T R39.2.1, Jazzy) linked over Gigabit Ethernet with CycloneDDS unicast (`ROS_DOMAIN_ID=10`); cross-host discovery and `jetson-head.service` active. | Simulated single-host execution in `ROS_DOMAIN_ID=42` / `77`. | Localhost fallback DDS profile (`cyclonedds_jetson.xml` fallback not yet deployed to Jetson). |
| **2D Navigation & Mapping** | Differential hoverboard base driver (`/odom` 50 Hz, battery telemetry), Hokuyo UST-05LX (`/scan` 40 Hz at `z=0.38 m` beam height), AMCL + Nav2 (`SmacPlanner2D` + `RotationShim` / `RegulatedPurePursuit`), live map reload via `nav_map_sync.sh` (`load_map` service). | Straight-line `navigate_to_pose` mock (`0.5 m/s`), raycast `/scan`, and Gazebo Harmonic differential drive model. | Semantic map & numbered object zone editor (`map_zone_editor.py`, 9 unit tests); offline D435 low-obstacle prototype (`low_obstacles.py`, not connected to Nav2). |
| **Arm Control & MoveIt 2 Manipulation** | `piper_arm_controller` deployed on Jetson (`md5 2e347c49`) with `auto_enable=False`, CAN feedback guard, `/arm/follow_joint_trajectory` action server (verified rejection when disabled and live CAN feedback `/arm/status` when energized). | MoveIt 2 (`base_athome_moveit_config`, TRAC-IK + OMPL, 117 arm-body + 18 internal collision pairs), multi-orientation grasp planner (0–30° pitch, 22 cm reach), target hold under 80% frame drop. | Full physical grasp series at 10/15/20 cm on real objects (**POR CONFIRMAR**); ArUco eye-in-hand calibration (`handeye.py` tested on synthetic data; physical calibration **POR CONFIRMAR**); table collision object in MoveIt scene. |
| **Wrist Camera & Object Detection** | Orbbec DaBai DC1 streaming (`~20–30 Hz`) and YOLOv8 TensorRT 10.16 FP16 on Jetson GPU (`~55 FPS` end-to-end, `/arm_camera/detections` received at `29.9 Hz` on NUC). | Synthetic projection of objects and bags into wrist camera RGB-D (`sim_scene.py`). | Open-vocabulary `yolov8s-worldv2` tested offline on RuleBook photos; TensorRT export on Jetson and real-object benchmark (**POR CONFIRMAR**). |
| **Pan-Tilt Head & RealSense D435** | RealSense D435 stable at `640x480 @ 15 Hz` on NUC. Dynamixel XL430 multi-turn unwrap fix and `pan_direction=+1` deployed on Jetson; **open hardware issue**: U2D2 USB disconnects under torque load (12 V power hub / USB cable under investigation). | Smooth 90°/s pan-tilt tracking, 2-row room scan (~16 s), and face centering in `base_athome_sim`. | Calibration and diagnostic script (`pan_tilt_calibrate.py`, 8 unit tests) awaiting full torque stability test (`M` / `V`) once 12 V supply is fixed. |
| **Speech Recognition (ASR) & HRI Receptionist** | RØDE VideoMic GO II + `whisper_node` (`base.en` + Silero VAD) running on Jetson; recognized short commands (~1.4 s latency) in initial tests, sensitive to continuous ambient noise. | Full HRI Receptionist flow (door wait, mutual introduction, gaze centering, 2-row scan, free-seat offer via YOLO-pose, guest web UI `:8090`). | Phonetic vocabulary matching and `VoiceClient` unit tests (12 ASR tests, 122 perception tests). Full end-to-end Receptionist run on physical hardware (**POR CONFIRMAR**). |
| **Torso Lift Column** | Hardware wired (NEMA 23 + ESP32-S3); **no physical motion test recorded yet** (homing and travel limits **POR CONFIRMAR**). | Prismatic `torso_lift_joint` simulated in URDF, MoveIt 2, and `sim_basura.sh`. | `torso_state` node (6 unit tests) and `stepper_controller.py` serial interface. |
| **Mission Executor, Commander & "Take Out the Trash"** | Start-signal LiDAR `door_open_detector` (`SIM_PASS` / `UNIT_PASS`; real Hokuyo arena door test **POR CONFIRMAR**). Commander voice/web pipeline has **never executed a physical mission on the real robot**. | `mission_executor` (`demo_fetch_bottle`), `sim_commander.sh` compound natural-language missions, and closed-loop `sim_basura.sh` (pointing gesture + 3D head/wrist `bag_vision` + floor grasp + `trash_drop` delivery, 1.6–5.6 cm TCP error in sim). | 76 skills/mission unit tests and 34 standalone `gpsr_parser` tests. Physical bag grasping and elevator descent on the real robot (**POR CONFIRMAR**). |
| **Meta Quest 2 WebXR Teleoperation** | USB `adb reverse` link and WebXR page verified on Jetson; currently paused. | 6-DOF Cartesian damped-least-squares IK + One Euro filter (`39.7 mm` tracked on `40 mm` step in `quest_teleop_simcheck.py`). | 18 unit tests (`quest_cartesian_core.py`); full closed-loop tracking test on the energized Piper arm (**POR CONFIRMAR**). |

---

## 8. License

License: **POR CONFIRMAR** (`package.xml` manifests in `base_athome_*` and `piper_msgs` declare `Apache-2.0`, `urg_node` declares `BSD`, and `hoverboard_base` / `piper_description` have pending license declarations; repository-wide root `LICENSE` file **POR CONFIRMAR**).
