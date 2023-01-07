DISCONTINUATION OF PROJECT

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project.  

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project.  

Contact: webadmin@linux.intel.com
# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 series) with ROS2.

**Note: If you want T265 supported, please check out [this branch](https://github.com/intel/ros2_intel_realsense/tree/refactor)**

## Installation Instructions

The following instructions were verified with ROS2 Dashing on **Ubutnu 18.04**.

### Dependencies
#### Install ROS2 packages [ros-dashing-desktop](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians)

#### Install ROS2 dependences
  ```bash
  sudo apt-get install ros-dashing-cv-bridge ros-dashing-librealsense2 ros-dashing-message-filters ros-dashing-image-transport
  ```
  Or you may install from sources:
  * [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
  * [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/librealsense.git) Currently, we support librealsense master branch.
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
  sudo apt-get install ros-dashing-realsense-camera-msgs ros-dashing-realsense-ros2-camera
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
source /opt/ros/dashing/setup.bash
colcon build --base-paths src/ros2_intel_realsense
```

## Usage Instructions

### Start the camera node
To start the camera node in ROS2, plug in the camera, then type the following command:

```bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/local_setup.bash
# To launch with "ros2 run"
ros2 run realsense_ros2_camera realsense_ros2_camera
# OR, to invoke the executable directly
realsense_ros2_camera
```

This will stream all camera sensors and publish on the appropriate ROS2 topics. PointCloud2 is enabled by default, till we provide ROS2 python launch options.

### Published Topics
Rectified depth image: [/camera/depth/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

Color image: [/camera/color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

Rectified infra1 image: [/camera/infra1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

Rectified infra2 image: [/camera/infra2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

Depth registered point cloud: [/camera/aligned_depth_to_color/color/points](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)

### Visualize Depth Aligned (i.e. Depth Registered) Point Cloud

To start the camera node in ROS2 and view the depth aligned pointcloud in rviz:
```bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/local_setup.bash
# console #1 launch rviz2
ros2 run rviz2 rviz2 -d realsense_ros2_camera/rviz/ros2.rviz
# console #2 launch realsense_ros2_camera
ros2 run realsense_ros2_camera realsense_ros2_camera
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

