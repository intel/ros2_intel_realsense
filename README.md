# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 series) with ROS2.

## Installation Instructions

The following instructions were verified with ROS2 Bouncy on **Ubutnu 18.04**.

### Step 1: Install the ROS2 dependenices
- #### Install [ROS2 Bouncy Bolson](https://index.ros.org/doc/ros2/Release-Bouncy-Bolson/) from binary or source.
- #### Source the environment
```bash
# from binary install
$ source /opt/ros/bouncy/setup.bash
# from source install
$ cd ~/ros2_ws
$ source install/local_setup.bash
```
- #### Install [ros2 cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2)
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/ros-perception/vision_opencv.git
$ git checkout ros2
$ cd ~/ros2_ws
$ colcon build --base-paths src/vision_opencv/cv_bridge
```

- #### Install [ros2 message_filter](https://github.com/ros2/message_filters)
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/ros2/message_filters.git
$ cd ~/ros2_ws
$ colcon build --base-paths src/message_filters
```

- #### Install [ros2 image_transport(from image_common)](https://github.com/ros-perception/image_common/tree/ros2)
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/ros-perception/image_common.git
$ git checkout ros2
$ cd ~/ros2_ws
$ colcon build --base-paths src/image_common/image_transport
```

### Step 2: Install the Intel&reg; RealSense&trade; SDK 2.0
- #### Install [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/tree/v2.9.1) from binary
Follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/v2.16.5/doc/distribution_linux.md).

- #### Install from source
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/IntelRealSense/librealsense.git
$ git checkout ros2debian
$ cd ~/ros2_ws
$ colcon build --base-paths src/librealsense
```

### Step 3: Install Intel&reg; RealSense&trade; ROS2 from Sources
- Goto an existing ros2 workspace, or [create one](https://github.com/ros2/ros2/wiki/Ament-Tutorial#create-directory-structure)
```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
```
- Clone the latest ros2_intel_realsense package into 'ros2_ws/src/'

```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/intel/ros2_intel_realsense.git
$ colcon build --base-paths src/ros2_intel_realsense
$ source ./install/local_setup.bash
```

## Usage Instructions

### Start the camera node
To start the camera node in ROS2, plug in the camera, then type the following command:

```bash
# To launch with "ros2 run"
$ ros2 run realsense_ros2_camera realsense_ros2_camera
# OR, to invoke the executable directly
$ realsense_ros2_camera
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
$ source ~/ros2_ws/install/local_setup.bash
$ realsense_ros2_camera

# console #2 launch rviz2
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run rviz2 rviz2
# add image and pointcloud2 and select topic in rviz.
```

This will launch [RViz](http://wiki.ros.org/rviz) and display the five streams: color, depth, infra1, infra2, pointcloud.

![realsense_ros2_camera visualization results](https://github.com/intel/ros2_intel_realsense/raw/master/realsense_ros2_camera/rviz/ros2_rviz.png "realsense_ros2_camera visualization results")

### Run tests
```Shell
$ colcon test --base-paths src/ros2_intel_realsense
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
