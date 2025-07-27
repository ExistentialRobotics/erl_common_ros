# erl_common_ros

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS1](https://img.shields.io/badge/ROS1-noetic-blue)](http://wiki.ros.org/)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue)](https://docs.ros.org/)

**erl_common_ros is the ROS wrapper of [erl_common](https://github.com/ExistentialRobotics/erl_common).**

## Features

- **Clock Synchronization**: Programmable clock node for simulation and data replay
- **Transform Broadcasting**: Convert transform messages to TF broadcasts
- **ROS Bridge Support**: Tools for bridging between ROS1 and ROS2
- **Bag File Conversion**: Scripts for converting ROS1 bags to ROS2 format
- **Cross-Platform**: Compatible with both ROS1 and ROS2

## Installation

### Prerequisites

- ROS1 Noetic or ROS2 Humble or later
- Ubuntu 20.04 for ROS1
- Ubuntu 22.04 or later for ROS2
- C++17 compatible compiler
- CMake 3.16 or higher

### Dependencies

This package depends on the following ERL packages:
- `erl_cmake_tools`
- `erl_common`

Standard ROS dependencies:
- `geometry_msgs`
- `tf2_ros`
- `rosgraph_msgs`

We also provide [docker files](docker/) for easy setup.

### Building from Source

1. **Clone the repository** into your ROS workspace:
   ```bash
   cd <your_ros_workspace>
   mkdir -p src
   vcs import --input https://raw.githubusercontent.com/ExistentialRobotics/erl_common_ros/main/erl_common_ros.repos src
   ```

2. **Build the package**:
   ```bash
   cd <your_ros_workspace>
   source /opt/ros/<distro>/setup.bash  # Replace <distro> with 'noetic' or 'humble'
   catkin build erl_common_ros  # For ROS1
   colcon build --packages-up-to erl_common_ros --cmake-args -DCMAKE_BUILD_TYPE=Release  # For ROS2
   ```

3. **Source the workspace**:
   ```bash
   cd <your_ros_workspace>
   source devel/setup.bash  # for ROS1
   source install/setup.bash  # for ROS2
   ```

## Available Nodes

### 1. `clock_node`

Publishes clock messages at a configurable rate for simulation time control.

**Published Topics:**
- `/clock` (rosgraph_msgs/Clock): Clock messages for simulation time

**Parameters:**
- `clock_rate` (double): Publishing frequency in Hz (default: 100.0)
- `start_time` (double): Starting time in seconds (default: current time)

**Usage:**
```bash
# ROS1
rosrun erl_common_ros clock_node _clock_rate:=50.0 _start_time:=0.0

# ROS2
ros2 run erl_common_ros clock_node --ros-args -p clock_rate:=50.0 -p start_time:=0.0
```

### 2. `transform_to_tf_node`

Subscribes to transform messages and broadcasts them as TF transforms.

**Subscribed Topics:**
- `transform` (geometry_msgs/TransformStamped): Transform messages to broadcast

**Parameters:**
- `transform_topic` (string): Input transform topic name (default: "transform")

**Usage:**
```bash
# ROS1
rosrun erl_common_ros transform_to_tf_node _transform_topic:=/my_transform

# ROS2
ros2 run erl_common_ros transform_to_tf_node --ros-args -p transform_topic:=/my_transform
```

## Utility Scripts

The package includes several utility scripts in the `scripts/` directory:

### 1. `convert_rosbag_1to2.bash`

Converts ROS1 bag files to ROS2 format using rosbags-convert.

**Usage:**
```bash
./scripts/convert_rosbag_1to2.bash input_bag.bag output_directory [topic1 topic2 ...]
```

### 2. `run_ros1_bridge.bash`

Launches the ROS1-ROS2 bridge for cross-version communication.

**Usage:**
```bash
./scripts/run_ros1_bridge.bash
```

### 3. `run_ros1_core.bash`

Starts a ROS1 core with appropriate environment setup.

**Usage:**
```bash
./scripts/run_ros1_core.bash
```
