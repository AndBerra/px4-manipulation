# PX4 Aerial Manipulation

![PX4](https://img.shields.io/badge/PX4-v1.16-blue)
![SITL](https://img.shields.io/badge/mode-SITL%20only-orange)
![Docker](https://img.shields.io/badge/docker-ready-green)

ROS 2 package for offboard attitude control of a PX4-based drone using interactive markers in RViz. Supports both direct interactive UI control and autonomous waypoint following.

This package extends the original work by Jaeyoung-Lim: https://github.com/Jaeyoung-Lim/px4-omnicopter

> [!Warning] 
> This package has been tested only in **SITL**

## Overview

The package provides a PD position controller running in PX4 offboard mode. The user can either control the drone interactively via an RViz marker or pre-record a waypoint list offline and have the drone follow it autonomously.

### Workflow

**Interactive mode (default)**

1. Start drone SITL with Gazebo from PX4 `make px4 sitl gz_omnicopter`
2. Open QGC and takeoff
3. Launch the manipulation node
   - it streams the offboard heartbeat and switches to offboard mode automatically after ~1 second
4. Use the RViz interactive marker to send pose commands to the drone via the `/set_pose` service

**Waypoint following mode**

1. Use the waypoint recorder tool to place waypoints offline in RViz and save them to `config/waypoints.json`
2. Start drone SITL with Gazebo from PX4 `make px4 sitl gz_omnicopter`
3. Open QGC and takeoff
4. Launch the waypoint following launch file — the node loads the waypoint list, switches to offboard mode, and begins sequencing through waypoints autonomously

---

## Package Structure

```bash
px4_manipulation/
├── config/                  # YAML parameter files, waypoints JSON and RVIZ config
├── launch/
│   ├── px4_manipulation.launch.py       # Interactive UI mode
│   ├── waypoint_following.launch.py     # Autonomous waypoint following
│   └── waypoint_recorder.launch.py      # Offline waypoint recording tool
├── src/
│   ├── main.cpp
│   └── px4_manipulation.cpp             # Main control node
├── include/
│   └── px4_manipulation/
│       └── px4_manipulation.h
└── Tools/
    ├── rviz_targetpose_marker.py        # Interactive marker UI
    ├── rviz_waypoint_recorder.py        # Waypoint recording tool
    └── rviz_waypoint_list_visualizer.py # Waypoint list visualizer
```

## ROS2 Interface

### Publishers

| Topic | Type | Description |
|---|---|---|
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Offboard heartbeat |
| `/fmu/in/vehicle_attitude_setpoint` | `px4_msgs/VehicleAttitudeSetpoint` | Attitude + thrust setpoint |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Arm / mode switch commands |
| `/waypoint_markers` | `visualization_msgs/MarkerArray` | Waypoint visualization in RViz |

### Subscribers

| Topic | Type | Description |
|---|---|---|
| `/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` | Navigation state |
| `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | Current attitude |
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | Current position and velocity |

### Services

| Service | Type | Description |
|---|---|---|
| `/set_pose` | `manipulation_msgs/SetPose` | Set target pose (interactive mode) |
| `/set_waypoints` | `manipulation_msgs/SetWaypoints` | Send waypoint list for sequencing |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kp` | double | 0.05 | Position proportional gain |
| `kd` | double | 0.05 | Velocity derivative gain |
| `hover_thrust` | double | 0.2 | Hover thrust feedforward |
| `follow_waypoints` | bool | false | Enable waypoint following mode |
| `waypoints_path` | string | "" | Path to waypoints JSON file |

## Setup

Clone into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/AndBerra/px4-aerial-manipulation.git
git clone --branch=v1.16 https://github.com/AndBerra/px4-offboard.git
git clone --branch=v1.16 https://github.com/PX4/px4-msgs.git
```

Install dependencies:

```bash
sudo apt install nlohmann-json3-dev
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select px4_manipulation
```

---

## Running

### 1. Start PX4 SITL

```bash
make px4_sitl gz_omnicopter
```

### 2. Start micro-ros-agent

```bash
MicroXRCEAgent udp4 --port 8888
```

### 3a. Interactive UI mode

Arm and take off from QGroundControl, then:

```bash
ros2 launch px4_manipulation px4_manipulation.launch.py
# optionally pass a custom config:
ros2 launch px4_manipulation px4_manipulation.launch.py config_file:=my_drone.yaml
```

Right-click the interactive marker in RViz → **Command Pose** to send a target position.

### 3b. Waypoint following mode

First record waypoints offline (no drone needed):

```bash
ros2 launch px4_manipulation waypoint_recorder.launch.py \
  waypoints_path:=/ros2_ws/src/px4-manipulation/px4_manipulation/config/waypoints.json
```

Use the RViz marker menu: **Add Waypoint** to place waypoints, **Save & Exit** when done.

Then arm and take off from QGroundControl, and launch:

```bash
ros2 launch px4_manipulation waypoint_following.launch.py \
  waypoints_path:=/ros2_ws/src/px4-manipulation/px4_manipulation/config/waypoints.json
```

The drone will switch to offboard mode automatically and follow the recorded waypoints in sequence, holding position at the last one when complete.

---

## Frame Convention

All internal computation is done in **NED (North-East-Down)**. External inputs (UI, waypoints JSON) use **ENU (East-North-Up)** and are converted at the boundary:

## Docker

To build and run inside Docker:

```bash
cd px4-manipulation/docker

# compile docker image
docker compose build

# start container and open a terminal shell inside
docker compose run --rm px4-manipulation bash
```

Inside the container the package is already compiled. You can directly launch the desired mode:

```bash
# Interactive UI mode
ros2 launch px4_manipulation px4_manipulation.launch.py

# Waypoint following mode
ros2 launch px4_manipulation waypoint_following.launch.py \
  waypoints_path:=/ros2_ws/src/px4-manipulation/px4_manipulation/config/waypoints.json
```

> [!Note] 
> PX4 SITL must be running on the host machine before launching the container. Make sure the container has network access to the host with `--network host` or via docker compose network settings.