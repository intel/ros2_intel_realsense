# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 series) with ROS2.

## Installation Instructions

The following instructions were verified with ROS2 Dashing on **Ubutnu 18.04**.

### Dependencies
#### Install ROS2 packages [ros-dashing-desktop](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/)
  ```bash
  sudo apt-get install ros-dashing-desktop
  ```
#### Install ROS2 dependences
  Currently, we support librealsense master branch.
  ```bash
  sudo apt-get install ros-dashing-cv-bridge ros-dashing-librealsense2 ros-dashing-message-filters ros-dashing-image-transport
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

### Install ros2_intel_realsense binary packages
  ```
  sudo apt-get install ros-dashing-realsense-msgs ros-dashing-realsense-ros
  ```
  The ros2_intel_realsense packages installation have been completed. You could jump to [Usage Instructions](https://github.com/intel/ros2_intel_realsense#usage-instructions) for executing, you could also install ros2_intel_realsense from source for more features. 

### Install ros2_intel_realsense from source
```bash
#get code
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_intel_realsense.git
git checkout refactor
#build
cd ~/ros2_ws
source /opt/ros/dashing/setup.bash
colcon build --symlink-install
```

## Usage Instructions

### Start the camera node
To start the camera node in ROS2, plug in the camera, then type the following command:

```bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/local_setup.bash
# To launch with "ros2 run", taking d435 for example:
ros2 run realsense_examples one_cam __ns:=/d435 __params:=~/ros2_ws/src/ros2_intel_realsense/realsense_ros/config/d435.yaml
# Or launch multiple cameras at one time, taking d435 and t265 for example:
ros2 run realsense_examples multi_cams __params:=~/ros2_ws/src/ros2_intel_realsense/realsense_ros/config/multi_cams.yaml
```
This will stream all camera sensors and publish on the appropriate ROS2 topics. PointCloud2 is enabled by default, till we provide ROS2 python launch options.

### Published Topics

*  D435  

[camera/color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/infra1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/infra2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/depth/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/pointcloud](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)

*  D435i  

[camera/color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/infra1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/infra2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/depth/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/pointcloud](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)

[camera/gyro/sample](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)

[camera/accel/sample](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)

*  T265  

[camera/fisheye1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/fisheye2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/gyro/sample](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)

[camera/accel/sample](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)

[camera/odom/sample](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg)


### Dynamically Reconfigure Parameters
Currently only support reconfigure parameters by `ros2 param` in runtime, e.g.
```
ros2 param list
ros2 param get <node_name> color0.enabled
ros2 param set <node_name> color0.enabled true
```
## Known Issues
* If you want to launch T265 and D435 camera at one time, you have to launch T265 first due to HW constraits which will be fixed in a future release of [librealsense](https://github.com/IntelRealSense/librealsense).
* Don't support launch node via launch file due to [issue#316](https://github.com/ros2/launch/issues/316) of launch package.
* Don't support dynamic reconfigure parameters by rqt_reconfigure due to [issue#53](https://github.com/ros-visualization/rqt_reconfigure/issues/53) of rqt_reconfigure, but you still can reconfigure parameters by `ros2 param`.
* We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit, but not support Mac OS X 10.12 (Sierra) and Windows 10 yet.

## Todo
* Add more tests
* Support diagnostics
* Support reconfigure parameters by GUI

## License
Copyright 2019 Intel Corporation

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

