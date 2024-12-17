<!-- Copyright 2024 TRAPS

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. -->

# [traps_example_ros](./README.md)

## Description

Communication and ROS2 bridge

## Usage

### Docker
```bash
docker run --rm --network host --ipc host -it ghcr.io/traps-robocup/comms_bridge_ros
```

### Build from source

#### Supported OS
- ubuntu 22.04

#### Requirement packages
- git
- ros-humble-ros-base
- asio
- colcon

#### 1. Clone
```bash
mkdir -p ~/ros2_ws/src && \
cd ~/ros2_ws/src && \
git clone https://github.com/TRAPS-RoboCup/comms_bridge_ros.git && \
cd ..
```
#### 2. Build
```bash
. /opt/ros/humble/setup.sh && colcon build
```