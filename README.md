# 🚨Gachon Police: AI-Based Road Situation Alert System

## Table of Contents

* [Project Overview](#project-overview)
* [Demo Video](#demo-video)
* [Key Features](#key-features)
* [System Architecture](#system-architecture)
* [System Requirements](#system-requirements)
* [Directory Structure](#directory-structure)
* [Installation and Build](#installation-and-build)

  * [1. Clone the Project](#1-clone-the-project)
  * [2. Host PC](#2-host-pc)
  * [3. Jetson Orin Nano](#3-jetson-orin-nano)
* [How to Use](#how-to-use)
* [Contributors](#contributors)

---

## Project Overview

Conventional road hazard alert systems, such as T map, heavily rely on centralized control centers and provide limited and insufficient contextual information about the situation. This project aims to overcome these limitations by developing an AI-based road situation alert system that understands the context of road environments by combining visual and textual information. It detects hazardous situations in real time, such as construction zones and traffic accidents.

The system is designed to operate on resource-constrained environments such as robots and vehicles using edge devices like the Jetson Orin Nano. By utilizing a VLM, it accurately recognizes dangers on the road and effectively communicates sensor data and inference results between ROS1 and ROS2 using a bridging mechanism.

Additionally, the system’s performance was verified through realistic scenarios in the MORAI SIM simulator. Finally, using WebSocket communication, the alert information both visual and audio is delivered to end users via an Android application.

**This project was conducted as part of the graduation project at the School of AI Software, Gachon University.**

---

## Demo Video
![image](https://github.com/user-attachments/assets/bd9ff7d7-df5b-4cc8-a05d-e1a6462d29a5)
[Demo Video using MORAI SIM](https://www.youtube.com/watch?v=CCFPlmEuvr4&list=PLYNk_CUWcIJAkZrfkXoM1lQpcdCJhrArQ&index=10)
[Demo Video using LIMO Robot](https://www.youtube.com/watch?v=l7TZe70wvsA&list=PLYNk_CUWcIJAkZrfkXoM1lQpcdCJhrArQ&index=15)

---

## Key Features

* Real-time detection of road hazards (construction, accidents, etc.)
* Context-aware reasoning using a VLM
* ROS1 and ROS2 bridge communication
* Edge device deployment (Jetson Orin Nano)
* Visual/audio alert delivery to Android application
* Scenario-based performance validation using MORAI SIM
  
---

## System Architecture

![image](https://github.com/user-attachments/assets/4ccbedd3-66bf-440d-9496-c23b8b3d1efd)

---

## System Requirements

| Component        | Specification                                     |
| ---------------- | ------------------------------------------------- |
| Host PC          | Ubuntu 20.04, MORAI SIM, ROS1 Noetic, Python ≥3.8 |
| Jetson Orin Nano | Ubuntu 22.04, JetPack 6.0, ROS2 Humble            |

---

## Directory Structure

```plaintext
. ── Project Root
├── application/                      # Android app and backend code
├── catkin_ws/src/                    # ROS1 (Noetic) workspace source cde
├── ros2_workspace/src/ros2_nanollm/  # ROS2 (Humble) VLM package
├── third_party/                      # Third-party libraries
└── .gitmodules                       # Git submodule settings
```
---

## Installation and Build

### 1. Clone the Project

```bash
git clone https://github.com/GuardiansOfGachon/guardians-of-gachon.git
cd guardians-of-gachon
```

> **Note**: You can use Git Sparse Checkout to clone only specific directories if needed.

### 2. Host PC

```bash
## Set ROS1 network environment variables

cd ~/catkin_ws
gedit ~/.bashrc
# Add the following lines:
# export ROS_MASTER_URI=http://<HOST_PC_IP>:11311
# export ROS_IP=<HOST_PC_IP>
# export ROS_HOSTNAME=<HOST_PC_IP>
source ~/.bashrc


## Install the MORAI ROS message package

cd ~/catkin_ws/src
git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install curl python3-pip net-tools ros-noetic-rosbridge-server ros-noetic-rosbridge-suite
pip3 install rospkg autobahn pymongo

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Jetson Orin Nano

```bash
## Install the ROS Bridge

cd ~/ros2_workspace
git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-build
cd ros-humble-ros1-bridge-builder/
docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
docker rmi ros-humble-ros1-bridge-builder
cd ros-humble-ros1-bridge
source /opt/ros/humble/setup.bash
source ~/ros2_workspace/ros-humble-ros1-bridge/install/local_setup.bash


## Clone the jetson-containers repository and run the install script

cd ~/Downloads
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh


## Launch the Jetson container with volume mounted to ros2_workspace

jetson-containers run -v ~/ros2_workspace:/ros2_workspace $(autotag nano_llm:humble)


## (Inside the container)

# Set ROS2 network environment variables
nano ~/.bashrc
# Add the following lines:
# export ROS_MASTER_URI=http://<YOUR HOST PC IP>:11311
# export ROS_IP=<YOUR DOCKER CONTAINER IP>
# export ROS_HOSTNAME=<YOUR DOCKER CONTAINER IP>
# export ROS_DOMAIN_ID=1
source ~/.bashrc

# Navigate to ROS2 workspace, build the project, and source the environment
cd ~/ros2_workspace
colcon build --symlink-install --base-paths src
bash /ros2_workspace/install/setup.bash
source /ros2_workspace/install/setup.bash
```

---

## How to Use

### 1. Run ROS Bridge

   * **Host PC**

     ```bash
     cd ~/catkin_ws
     source devel/setup.bash
     roscore &
     roslaunch rosbridge_server rosbridge_websocket.launch
     ```
   * **Jetson Orin Nano**

     ```bash
     ## (Inside the container)
     
     cd ~/ros2_workspace
     source install/setup.bash
     source ros-humble-ros1-bridge/install/local_setup.bash
     ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
     ```

### 2. Start VLM Inference (in Jetson Orin Nano)

   ```bash
   ## (Inside the container)

   cd ~/ros2_workspace
   source install/setup.bash
   ros2 launch ros2_nanollm camera_input_example.launch.py
   ```

### 3. Start WebSocket Server (in Jetson Orin Nano)
   
   ```bash
   ## (Inside the container)
   
   cd ros2_workspace
   sudo apt update
   sudo apt install ros-humble-rosbridge-server
   source /opt/ros/humble/setup.bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

### 4. Global Path Tracking (in Host PC)

   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   rosrun global_path_tracking global_path_pub.py
   rosrun global_path_tracking gpsimu_parser.py
   rosrun global_path_tracking local_path_pub.py
   rosrun global_path_tracking acc_advanced_purepursuit.py /local_path
   python3 src/publish_vehicle_location.py
   ```

### 5. Run Android Application (in Host PC)
   * Open the `application/` folder in Android Studio.
   * Connect a device or start an emulator.
   * Click ▶ Run to launch the app.

---

## Contributors

* Dahye Kim ([GitHub](https://github.com/dahye411))
* Minseo Lee ([GitHub](https://github.com/minxxeo))
* Saewon Min ([GitHub](https://github.com/saewonalice))
* Songeon Kim ([GitHub](https://github.com/PoCoToday))
 
---

