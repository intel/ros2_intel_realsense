# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 series) with ROS2.

## Installation Instructions

The following instructions were verified with ROS2 crystal on **Ubutnu 18.04**.

### Dependencies
#### Install ROS2 packages [ros-crystal-desktop](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/)
  ```bash
  sudo apt-get install ros-crystal-desktop
  ```
  The ros-crystal-desktop will include below packages.
  * ament_cmake
  * std_msgs
  * sensor_msgs
  * builtin_interfaces
  * eigen
  * rmw_implementation
  * rclcpp
  * tf2_ros
  * rosidl_default_generators
  * rosidl_default_runtime

#### Install ROS2 dependences
  Currently, we support librealsense master branch.
  ```bash
  sudo apt-get install ros-crystal-cv-bridge ros-crystal-librealsense2 ros-crystal-message-filters ros-crystal-image-transport
  ```
  * [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
  * [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/librealsense.git)
  * [ros2_message_filters](https://github.com/ros2/message_filters)
  * [ros2 image_transport](https://github.com/ros-perception/image_common/tree/ros2)
  
#### Install Other non-ROS debian packages
  ```
  sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
  sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  ```
  * libssl-dev
  * libusb-1.0-0-dev
  * pkg-config
  * libgtk-3-dev
  * libglfw3-dev
  * libgl1-mesa-dev
  * libglu1-mesa-dev

### Install ros2_intel_realsense binary packages
  ```
  sudo apt-get install ros-crystal-realsense-camera-msgs ros-crystal-realsense-ros2-camera
  ```
  The ros2_intel_realsense packages installation have been completed. You could jump to [Usage Instructions](https://github.com/intel/ros2_intel_realsense#usage-instructions) for executing, you could also install ros2_intel_realsense from source for more features. 

### Install ros2_intel_realsense from source
```bash
#get code
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_intel_realsense.git

#build
cd ~/ros2_ws
source /opt/ros/crystal/setup.bash
colcon build --base-paths src/ros2_intel_realsense
```

## Usage Instructions

### Start the camera node
To start the camera node in ROS2, plug in the camera, then type the following command:

```bash
source /opt/ros/crystal/setup.bash
source ~/ros2_ws/install/local_setup.bash
# To launch with "ros2 run"
ros2 run realsense_ros2_camera realsense_ros2_camera
# OR, to invoke the executable directly
realsense_ros2_camera
```

This will stream all camera sensors and publish on the appropriate ROS2 topics. PointCloud2 is enabled by default, till we provide ROS2 python launch options.

### Published Topics
[/camera/depth/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/infra1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/infra2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/depth/color/points](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)

### Visualize Depth Point Cloud

To start the camera node in ROS2 and view the depth pointcloud in rviz:
```bash
# console #1 launch realsense_ros2_camera
source /opt/ros/crystal/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 launch realsense_ros2_camera ros2_intel_realsense.launch.py
```

This will launch [RViz](http://wiki.ros.org/rviz) and display the five streams: color, depth, infra1, infra2, pointcloud.

![realsense_ros2_camera visualization results](https://github.com/intel/ros2_intel_realsense/raw/master/realsense_ros2_camera/rviz/ros2_rviz.png "realsense_ros2_camera visualization results")

### Run tests
```Shell
colcon test --base-paths src/ros2_intel_realsense
```

## Known Issues
* This ROS2 node does not currently provide any dynamic reconfigure support for camera properties/presets.
* We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit, but not support Mac OS X 10.12 (Sierra) and Windows 10 yet.

## Todo
A few features to be ported from the latest ROS [realsense](https://github.com/intel-ros/realsense.git)
* Preset/Controls

## License
Copyright 2018 Intel Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this project except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

**Other names and brands may be claimed as the property of others*

Any security issue should be reported using process at https://01.org/security

