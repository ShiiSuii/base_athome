# Base@Home (base_athome)

Autonomous Mobile Manipulation and Human-Robot Interaction (HRI) platform designed for the **RoboCup @Home** international competition (Open Platform League - OPL).

The robot features a high-traction differential drive base powered by dual hoverboard hub motors with closed-loop Hall odometry, a 2D Hokuyo LiDAR, a 1-DOF motorized vertical elevator column (NEMA 23), a 6-DOF AgileX Piper robotic arm, an interactive expressive animated face screen, and a dual-axis pan-tilt head equipped with an Intel RealSense D435 RGB-D camera and directional RØDE audio system.

---

## Table of Contents

1. [System Architecture & Distributed Computing](#1-system-architecture--distributed-computing)
2. [Hardware Components & Pinout](#2-hardware-components--pinout)
3. [Kinematic Structure & TF Tree](#3-kinematic-structure--tf-tree)
4. [Autonomous Navigation (Nav2 Stack)](#4-autonomous-navigation-nav2-stack)
5. [2D & 3D Mapping (SLAM Toolbox & RTAB-Map)](#5-2d--3d-mapping-slam-toolbox--rtab-map)
6. [RoboCup @Home HRI Receptionist Challenge](#6-robocup-home-hri-receptionist-challenge)
7. [Manipulator & Gripper (AgileX Piper 6-DOF)](#7-manipulator--gripper-agilex-piper-6-dof)
8. [Torso Elevator (NEMA 23 Linear Rail)](#8-torso-elevator-nema-23-linear-rail)
9. [Installation & Build Guide](#9-installation--build-guide)
10. [Step-by-Step Launch & Operation Guide](#10-step-by-step-launch--operation-guide)
11. [Troubleshooting & Architectural Decisions](#11-troubleshooting--architectural-decisions)

---

## 1. System Architecture & Distributed Computing

The robot employs a distributed dual-computer architecture connected over a private Gigabit Ethernet network using **ROS 2 CycloneDDS** (`ROS_DOMAIN_ID=10`):

```
+-----------------------------------------------------------------------------------------+
|                           INTEL NUC (10.42.0.1 - ROS 2 Jazzy)                           |
|                                                                                         |
|  +--------------------+    USB 3.0 (/dev/video*)    +--------------------------------+  |
|  |  realsense2_camera |<===========================>|  Intel RealSense D435 (RGB-D)  |  |
|  +---------+----------+                             +--------------------------------+  |
|            | /camera/camera/color/image_raw, /camera/camera/depth/image_rect_raw        |
|            v                                                                            |
|  +--------------------+   Ethernet (192.168.0.10)   +--------------------------------+  |
|  |      urg_node      |<===========================>|     Hokuyo UST-05LX LiDAR      |  |
|  +---------+----------+                             +--------------------------------+  |
|            | /scan                                                                      |
|            v                                                                            |
|  +--------------------+    UART (/dev/ttyUSB0)      +--------------------------------+  |
|  |  hoverboard_driver |<===========================>| Dual 6.5" Hoverboard Hubs      |  |
|  +---------+----------+                             +--------------------------------+  |
|            | /odom, /tf                                                                 |
|            v                                                                            |
|  +--------------------+                             +--------------------------------+  |
|  |     Nav2 Stack     |                             |  RoboCup @Home HRI Perception  |  |
|  | (Smac, Shim + RPP) |                             |  - SFace 128-D Biometrics      |  |
|  |  Footprint: 0.44m  |                             |  - YOLOv8-pose Seating Detect  |  |
|  |  Inflation: 0.59m  |                             |  - Receptionist State Machine  |  |
|  +--------------------+                             +--------------------------------+  |
|                                                                                         |
|            ^ CycloneDDS Unicast Peering (ROS_DOMAIN_ID=10)                              |
+------------|----------------------------------------------------------------------------+
             | Gigabit Ethernet (10.42.0.0/24)
+------------|----------------------------------------------------------------------------+
|            v                                                                            |
|  +--------------------+    UART (/dev/ttyU2D2)      +--------------------------------+  |
|  |   pan_tilt_node    |<===========================>| ROBOTIS Dynamixel XL430 Servos |  |
|  +---------+----------+    1 Mbps Protocol 2.0      +--------------------------------+  |
|            | /joint_states, /pan_tilt/target_degrees                                    |
|            |                                                                            |
|  +---------+----------+    USB Audio (hw:0,0)       +--------------------------------+  |
|  |  whisper_asr_node  |<===========================>| RODE VideoMic GO II (Mic)      |  |
|  +---------+----------+    48kHz -> 16kHz VAD       +--------------------------------+  |
|            | /voice/transcription, /voice/status, /voice/mic_level                      |
|            |                                                                            |
|  +---------+----------+    CAN Bus (can0/can1)      +--------------------------------+  |
|  | piper_arm_ctrl     |<===========================>| AgileX Piper 6-DOF Manipulator |  |
|  +---------+----------+                             +--------------------------------+  |
|            |                                                                            |
|  +---------+----------+    USB 2.0 (/dev/video*)    +--------------------------------+  |
|  |  stream_orbbec.py  |<===========================>| Orbbec DaBai DC1 (Wrist Cam)   |  |
|  +--------------------+                             +--------------------------------+  |
|                                                                                         |
|                    NVIDIA JETSON ORIN NANO (10.42.0.12 - ROS 2 Humble)                  |
+-----------------------------------------------------------------------------------------+
```

### CycloneDDS Configuration
Both hosts utilize explicit unicast peer discovery configured via `~/cyclonedds.xml` to prevent UDP broadcast storms and ensure stable inter-host communication:
* **Intel NUC IP:** `10.42.0.1` (`enp1s0`)
* **Jetson IP:** `10.42.0.12` (`enP8p1s0`)
* **ROS Domain ID:** `ROS_DOMAIN_ID=10`
* **Transport:** Unicast UDP with multicast disabled on point-to-point links.

---

## 2. Hardware Components & Pinout

| Subsystem | Hardware Component | Interface / Port | Parameters / Protocol |
| :--- | :--- | :--- | :--- |
| **Main Compute (NUC)** | 12th Gen Intel Core i3-1220P, 16GB RAM | Onboard | Ubuntu 24.04 LTS + ROS 2 Jazzy |
| **Edge Compute (Jetson)**| NVIDIA Jetson Orin Nano (8GB) | Onboard | Ubuntu 22.04 LTS + ROS 2 Humble |
| **Mobile Base Motors** | Dual 6.5" Brushless Hoverboard Hub Motors | UART via USB-TTL (`/dev/ttyUSB0`) | 115200 baud, 50 Hz control loop |
| **2D Navigation LiDAR** | Hokuyo UST-05LX (0.02 - 5.0m, 270 deg) | Gigabit Ethernet (`192.168.0.10`) | SCIP 2.0 protocol @ 40 Hz |
| **Head RGB-D Camera** | Intel RealSense D435 | USB 3.2 Gen 1 (Direct NUC USB) | 640x480 @ 15 FPS (Color + Aligned Depth) |
| **Pan-Tilt Mechanism** | 2x ROBOTIS Dynamixel XL430-W250-T | U2D2 UART (`/dev/ttyU2D2`) on Jetson | 1,000,000 baud, Protocol 2.0 |
| **Audio Capture** | RØDE VideoMic GO II Shotgun Microphone | USB Audio (ALSA `hw:0,0`) on Jetson | 48 kHz native capture, WebRTC VAD |
| **Audio Output** | 3.5mm Analog Audio / USB DAC | Stereo Audio Line-out | Piper Neural TTS + WAV turn-taking cue |
| **Manipulator** | AgileX Piper 6-DOF Lightweight Arm | CAN Bus (`can0` / `can1`) on Jetson | 500 kbps / 1 Mbps CAN, 1.5 kg payload |
| **Wrist Camera** | Orbbec DaBai DC1 Depth Sensor | USB 2.0 on Jetson | Eye-in-Hand camera for table manipulation |
| **Torso Elevator** | NEMA 23 Stepper Motor + Lead Screw | USB Serial (`/dev/ttyUSB1`) on NUC | ESP32-S3 pulse controller @ 115200 baud |

---

## 3. Kinematic Structure & TF Tree

The URDF tree is defined in [`src/hoverboard_base/urdf/robot.urdf.xacro`](file:///home/athome/base_athome_ws/src/hoverboard_base/urdf/robot.urdf.xacro):

```
map
 └── odom (Published by nav2_amcl or slam_toolbox)
      └── base_footprint (Published by hoverboard_driver @ 50 Hz)
           └── base_link
                ├── left_wheel (Continuous rotation, Y-axis)
                ├── right_wheel (Continuous rotation, Y-axis)
                ├── rear_caster (Passive omnidirectional support)
                ├── laser (Hokuyo UST-05LX, x=+0.20m, z=+0.18m)
                └── torso_column (Central vertical extruded column, x=0.0m, y=0.0m)
                     └── torso_lift_link (Elevator carriage, adjustable z=0.30m .. 1.20m)
                          ├── mounting_cube (15x15x15 cm structural carriage)
                          │    └── arm_base_link (AgileX Piper 6-DOF arm base, offset to left)
                          │         └── link1 .. link6 -> piper_gripper_base
                          │              └── orbbec_wrist_camera_link (Eye-in-Hand)
                          ├── face_display_link (Interactive emotion screen)
                          └── pan_tilt_base_link
                               └── pan_link (Dynamixel ID 1 - Yaw, Z-axis: -65 deg to +65 deg)
                                    └── tilt_link (Dynamixel ID 2 - Pitch, Y-axis: -35 deg to +35 deg)
                                         └── camera_link (Intel RealSense D435)
                                              ├── camera_color_optical_frame
                                              └── camera_depth_optical_frame
```

### Physical Dimensions & Calibration
* **Track Width:** `0.5620 m`
* **Effective Wheel Radius:** `0.0853 m`
* **Robot Footprint Radius (`robot_radius`):** `0.44 m` (optimized for tight domestic doorways)
* **Costmap Inflation Radius (`inflation_radius`):** `0.59 m`
* **Dynamixel Zero Calibration:**
  * **Pan (ID 1):** `2059` ticks (~180.0 deg electrical center)
  * **Tilt (ID 2):** `3932` ticks (~345.0 deg electrical center, horizon level)

---

## 4. Autonomous Navigation (Nav2 Stack)

Configured in [`src/base_athome_navigation/config/nav2_params.yaml`](file:///home/athome/base_athome_ws/src/base_athome_navigation/config/nav2_params.yaml).

### Controller Architecture for Scrub Rubber Tires
Hoverboard solid rubber tires have high static friction (**stiction**), which frequently causes classical controllers (such as standard DWB or pure pursuit) to stall, chatter, or fail when rotating in place. The system solves this via a layered hybrid controller:

1. **`RotationShimController` (Primary Heading Governor):**
   * Wraps `RegulatedPurePursuitController`.
   * When path heading deviation exceeds `0.785 rad` (45 degrees), the shim takes control, zeroes linear velocity, and executes an exact in-place rotation at `1.0 rad/s`.
   * **Exact Final Goal Yaw Alignment (`rotate_to_goal_heading: true`):** When the robot reaches `xy_goal_tolerance: 0.18 m`, the shim rotates the base until `yaw_goal_tolerance: 0.10 rad` (~5.7 deg) is achieved.
2. **`RegulatedPurePursuitController` (Path Following):**
   * `min_lookahead_dist: 0.58 m` (strictly larger than track width `0.562 m` to eliminate high-frequency steering oscillation).
   * `desired_linear_vel: 0.38 m/s` (smooth, controlled domestic speed).
   * `min_approach_linear_velocity: 0.16 m/s`.
3. **Driver Stiction Breakaway Floors ([`hoverboard_driver.py`](file:///home/athome/base_athome_ws/src/hoverboard_base/hoverboard_base/hoverboard_driver.py)):**
   * Enforces minimum torque floor (`min_speed_raw: 38`, `min_steer_raw: 45`) on non-zero commands to break static friction without jerking.
   * Includes an S-curve slew rate filter (max 8 units/cycle linear, 12 units/cycle angular @ 50 Hz) preventing motor voltage spikes.

---

## 5. 2D & 3D Mapping (SLAM Toolbox & RTAB-Map)

### 2D Laser SLAM (Fast Floorplan Generation)
```bash
ros2 launch base_athome_navigation slam.launch.py
```
To save the generated map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 3D Multimodal Mapping (LiDAR + RealSense D435)
Combines 2D laser scan graph SLAM with dense 3D visual keyframes from the RealSense D435 camera mounted on the pan-tilt head:
```bash
ros2 launch base_athome_mapping rtabmap_realsense.launch.py
```
* Generates persistent global point clouds in `/rtabmap/cloud_map` (voxelized to 3 cm).
* Stores keyframes in SQLite database `~/.ros/rtabmap.db`.
* Export to textured 3D mesh (`.ply` / `.obj`):
  ```bash
  rtabmap-export ~/.ros/rtabmap.db
  ```

---

## 6. RoboCup @Home HRI Receptionist Challenge

Implemented in [`src/base_athome_perception/scripts/hri_receptionist_challenge.py`](file:///home/athome/base_athome_ws/src/base_athome_perception/scripts/hri_receptionist_challenge.py).

### Challenge Objective
According to the official RoboCup @Home Rulebook, the robot must:
1. Wait at the entrance door for arriving guests.
2. Detect the guest, track their face with the pan-tilt head, and greet them.
3. Inquire their name and favorite drink using natural voice interaction.
4. Enroll the guest's face into its biometric database with quality filtering.
5. Guide the guest into the living room.
6. Locate the host ("Charlie"), introduce the new guest to the host, and introduce previous guests.
7. Inspect available seating (sofas, chairs) using human pose estimation to find an empty seat.
8. Point with the 6-DOF Piper arm toward the empty seat and offer it to the guest.
9. Return to the door to receive the next guest.

```
       +-----------------------+
       |      STATE_INIT       |
       +-----------+-----------+
                   | Pre-flight checks (Camera, Nav2, Audio)
                   v
       +-----------------------+
       |   STATE_NAV_TO_DOOR   |
       +-----------+-----------+
                   | Arrived at Waypoint 'A' / 'door'
                   v
       +-----------------------+
       |    STATE_WAIT_DOOR    |<-------------------------------+
       +-----------+-----------+                                |
                   | Multimodal detection (Face >= 500px, 1.0s debounced)
                   v                                            |
       +-----------------------+                                |
       | STATE_GREET_AND_PROMPT|                                |
       +-----------+-----------+                                |
                   | Say greeting + play turn-taking beep       |
                   v                                            |
       +-----------------------+                                |
       | STATE_WAIT_NAME_DRINK |                                |
       +-----------+-----------+                                |
                   | NLP extraction (Name & Drink)              |
                   v                                            |
       +-----------------------+                                |
       | STATE_CONFIRM_DETAILS |                                |
       +-----------+-----------+                                |
                   | Polite confirmation                        |
                   v                                            |
       +-----------------------+                                |
       |  STATE_MEMORIZE_FACE  |                                |
       +-----------+-----------+                                |
                   | 10 high-quality biometric frames           |
                   v                                            |
       +-----------------------+                                |
       |STATE_NAV_TO_LIVING_RM |                                |
       +-----------+-----------+                                |
                   | Arrived at Waypoint 'B' / 'living_room'    |
                   v                                            |
       +-----------------------+                                |
       |    STATE_FIND_HOST    |                                |
       +-----------+-----------+                                |
                   | Pan-tilt scan & facial match for "Charlie" |
                   v                                            |
       +-----------------------+                                |
       |STATE_INTRODUCE_GUESTS |                                |
       +-----------+-----------+                                |
                   | Formal introduction of name and drink      |
                   v                                            |
       +-----------------------+                                |
       |DETECT_AND_OFFER_SEAT  |                                |
       +-----------+-----------+                                |
                   | YOLOv8-pose seat inspection + Arm Pointing |
                   v                                            |
       +-----------------------+                                |
       | STATE_CHECK_NEXT_GUEST+--------------------------------+
       +-----------+-----------+ More guests remaining
                   | All guests served
                   v
       +-----------------------+
       |STATE_CHALLENGE_COMPLET|
       +-----------------------+
```

### Key Technical Subsystems

#### 1. Biometric Face Recognition (`FaceRecognizer`)
* **Detection:** OpenCV DNN YuNet / SSD ResNet-10.
* **Feature Extraction:** SFace (`face_recognition_sface_2021dec.onnx`) embedding faces into a 128-dimensional unit hypersphere.
* **Quality Filter:** Laplacian variance sharpness check ($\sigma^2 > 100.0$) ensures only sharp, motion-blur-free images are enrolled.
* **Matching:** Cosine similarity threshold $\ge 0.363$.
* **Persistence:** Profiles stored in [`profiles.json`](file:///home/athome/base_athome_ws/src/base_athome_perception/config/faces_database/profiles.json) and sessions in [`receptionist_session.json`](file:///home/athome/base_athome_ws/src/base_athome_perception/config/faces_database/receptionist_session.json).

#### 2. Speech & Audio Architecture
* **Capture:** RØDE VideoMic GO II shotgun microphone running 48 kHz native capture on the Jetson.
* **VAD & Anti-Echo:** WebRTC Voice Activity Detector + dynamic RMS energy threshold. Automatic mute gating (`/voice/mute`) ensures the robot never transcribes its own voice.
* **ASR Model:** `faster-whisper` (`tiny` multilingual int8 model on CPU) providing ~200 ms transcription latency for English and Spanish.
* **NLP Entity Parser:** Regex and fuzzy dictionary parsing for common English/Spanish names and beverages (Coke, Water, Juice, Beer, Wine, Tea, Coffee, Milk) with polite clarification fallback.
* **Turn-taking Acoustic Cue:** Plays an acoustic prompt chime (`beep.wav`) whenever opening the microphone.

#### 3. Human Pose & Seating Inspection
* **Detector:** YOLOv8-pose (`yolov8n-pose.pt`) running keypoint inference.
* **Biomechanical Logic:** Evaluates knee-to-hip and torso vertical vectors to classify persons as `Sitting` vs `Standing`. Matches persons to semantic seat zones (`sof_principal`, `chair_1`, `chair_2`) to detect vacant spots.

#### 4. Expressive Facial Display & Arm Gestures
* **Face Screen:** Publishes emotion state strings to `/robot/emotion` (`happy`, `speaking`, `listening`, `thinking`, `neutral`) synchronized with the animated OLED/HDMI display.
* **Arm Gestures:** Controls the Piper 6-DOF arm to execute deictic pointing gestures (`point_couch`, `point_chair`) and social greeting gestures (`wave_hello`, `cube_stow`).

#### 5. Interactive HUD Keybindings
While the challenge is running, the OpenCV GUI accepts interactive test inputs:

| Key | Function |
| :--- | :--- |
| **`[SPACE]` / `[ENTER]`** | Manually force step forward / simulate detection or sensor input |
| **`[T]`** | Type simulated guest response via keyboard (bypasses microphone) |
| **`[R]`** | Force trigger microphone listening window (`/voice/listen_trigger`) |
| **`[G]`** | Test Piper arm deictic pointing gesture on living room seats |
| **`[N]`** | Skip current Nav2 navigation goal (simulates instant arrival) |
| **`[Q]` / `[ESC]`** | Clean shutdown, stow arm, and save session memory to disk |

---

## 7. Manipulator & Gripper (AgileX Piper 6-DOF)

The AgileX Piper is a lightweight 6-DOF robotic arm (1.5 kg payload, 626 mm reach) mounted on the structural carriage cube on the left side of the torso elevator.

* **CAN Bus Interface:** Operates on `can0` / `can1` @ 500 kbps via SocketCAN (`gs_usb` adapter).
* **Wrist Eye-in-Hand Camera:** Orbbec DaBai DC1 mounted on the wrist for table-top object grasping.
* **Pre-configured Gesture Poses (Joint Degrees `[j1..j6]`):**
  * `home` / `cube_stow`: `[72.09, 176.50, -155.15, 102.62, -31.51, -99.91]`
  * `point_couch`: `[45.00, 155.00, -75.00, 50.00, -50.00, -80.00]`
  * `point_chair`: `[25.00, 150.00, -70.00, 35.00, -45.00, -80.00]`
  * `wave_hello`: `[60.00, 130.00, -110.00, 80.00, -10.00, -90.00]`

---

## 8. Torso Elevator (NEMA 23 Linear Rail)

The vertical torso is actuated by a high-torque NEMA 23 stepper motor paired with a lead screw mechanism. Controlled by an ESP32-S3 over USB serial:

```bash
# Launch interactive stepper control utility
python3 /home/athome/base_athome_ws/src/base_athome_bringup/scripts/stepper_controller.py
```
* **Supported Commands:** `home`, `top`, `bottom`, `rpm <val>`, `fast`, `normal`, `slow`.

---

## 9. Installation & Build Guide

### Prerequisites
* **Operating Systems:**
  * Intel NUC: Ubuntu 24.04 LTS (Noble) with **ROS 2 Jazzy Jalisco**.
  * NVIDIA Jetson: Ubuntu 22.04 LTS (Jammy) with **ROS 2 Humble Hawksbill**.
* **Essential System Packages (NUC):**
  ```bash
  sudo apt update && sudo apt install -y \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-realsense2-camera \
      ros-jazzy-librealsense2 \
      ros-jazzy-cv-bridge \
      ros-jazzy-urg-node \
      ros-jazzy-rmw-cyclonedds-cpp \
      python3-pip python3-colcon-common-extensions
  ```

### Python Vision Virtual Environment
Neural network inference (YOLOv8, SFace, Faster-Whisper, PyTorch) runs in a dedicated virtual environment:
```bash
python3 -m venv ~/ros2_vision_venv --system-site-packages
source ~/ros2_vision_venv/bin/activate
pip install ultralytics faster-whisper webrtcvad sounddevice opencv-python pyyaml numpy
```

### Building the Workspace
```bash
cd ~/base_athome_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 10. Step-by-Step Launch & Operation Guide

### Step 1: Base Platform & Navigation Bringup
In Terminal 1 on the NUC, launch the motor driver, LiDAR, URDF state publisher, and Nav2 stack:
```bash
source /opt/ros/jazzy/setup.bash
source ~/base_athome_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch base_athome_navigation nav2_bringup.launch.py
```

### Step 2: Head Subsystems (Pan-Tilt & Audio on Jetson)
The Jetson runs a systemd service (`jetson-head.service`) automatically on boot. To verify or manage it:
```bash
ssh ubuntu@10.42.0.12 "sudo systemctl status jetson-head.service"
```
*(The Jetson manages Pan-Tilt servos on `/dev/ttyU2D2` and Whisper ASR on the RØDE USB microphone, publishing `/voice/transcription` and `/voice/mic_level` over CycloneDDS).*

### Step 3: Run the RoboCup @Home Receptionist Challenge
In Terminal 2 on the NUC, launch the full Receptionist Challenge:
```bash
source /opt/ros/jazzy/setup.bash
source ~/base_athome_ws/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch base_athome_perception hri_receptionist.launch.py
```
*(This automatically spawns the RealSense D435 camera driver on the NUC at 15 FPS, connects to Nav2, and opens the interactive OpenCV HUD).*

Alternatively, run the Python script directly:
```bash
/home/athome/ros2_vision_venv/bin/python3 ~/base_athome_ws/src/base_athome_perception/scripts/hri_receptionist_challenge.py
```

---

## 11. Troubleshooting & Architectural Decisions

### 1. Why is the RealSense D435 connected directly to the NUC instead of the Jetson?
* **Bandwidth:** Streaming uncompressed 640x480x15 FPS RGB-D data requires **~112 Mbps** continuously. When Nav2 is navigating, global/local costmaps, laser scans (40 Hz), and TF transforms flooded the Jetson-NUC Ethernet link, causing UDP packet drops and freezing the video stream.
* **DDS Serialization Differences:** Ubuntu 22.04 (ROS 2 Humble) and Ubuntu 24.04 (ROS 2 Jazzy) have known serialization differences in CycloneDDS (`serdata.cpp:384: string data is not null-terminated`), causing the camera publisher on Humble to stall when interacting with Jazzy Nav2 nodes.
* **USB Electrical Stability:** The RealSense D435 draws up to 1.5A peak. Together with Pan-Tilt servos, the RØDE microphone, CAN adapter, and DaBai camera, the Jetson USB bus suffered brownout resets during motor acceleration. Connecting the camera to the NUC's dedicated USB 3.0 controller eliminated this completely.

### 2. Camera Stagnation / Freezing Diagnosis
If the HUD ever displays `ALERTA: CAMARA SIN DATOS`:
1. Verify RealSense enumeration on the NUC:
   ```bash
   lsusb | grep 8086:0b07
   ```
2. Verify that `/camera/camera/color/image_raw` is publishing:
   ```bash
   ros2 topic hz /camera/camera/color/image_raw
   ```
   *(Expected rate: `14.99 Hz` with `std dev < 0.003s`).*

### 3. Microphone Audio & Whisper ASR Check
To monitor audio health and microphone input level:
```bash
ros2 topic echo /voice/mic_level
# Talk into the microphone; RMS level should climb from ~0.005 (ambient) to ~0.08+
ros2 topic echo /voice/transcription
```
To force a test speech recognition window manually:
```bash
ros2 service call /voice/listen_trigger std_srvs/srv/Trigger
```

---

## Authors & License

Developed for the **RoboCup @Home** Open Platform League.  
Licensed under the Apache License 2.0.
