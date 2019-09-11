# ROS2 Package for Intel&reg; RealSense&trade; Devices
## Supported Devices
* Intel&reg; RealSense&trade; Camera D400-Series
   - Intel&reg; RealSense&trade; Depth Cameras D415, D435 and D435i
* Intel&reg; RealSense&trade; Tracking Camera T265

## Installation Instructions

The following instructions were verified with ROS2 Dashing on Ubuntu 18.04.

### Install Dependencies
#### 1. Install ROS2 packages [ros-dashing-desktop](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/)

#### 2. Install ROS2 dependences
  Currently, we support librealsense master branch.
  ```bash
  sudo apt-get install ros-dashing-cv-bridge ros-dashing-librealsense2 ros-dashing-message-filters ros-dashing-image-transport
  ```
  * [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/librealsense.git)
  * [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
  * [ros2_message_filters](https://github.com/ros2/message_filters)
  * [ros2 image_transport](https://github.com/ros-perception/image_common/tree/ros2)
  
#### 3. Install other non-ROS debian packages
  ```
  sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
  sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  ```
<!--
### Install ros2_intel_realsense binary packages
  ```
  sudo apt-get install ros-dashing-realsense-msgs ros-dashing-realsense-ros
  ```
  The ros2_intel_realsense packages installation have been completed. You could jump to [Usage Instructions](https://github.com/intel/ros2_intel_realsense#usage-instructions) for executing, you could also install ros2_intel_realsense from source for more features. -->

### Install ros2_intel_realsense From Source
```bash
# fetch code
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/intel/ros2_intel_realsense.git
cd ros2_intel_realsense
git checkout refactor
# build
source /opt/ros/dashing/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
```

## Usage Instructions

### Start Camera Node
Obtain the serial number of your device
```bash
rs-enumerate-devices
```
Change the corresponding yaml file with the specific serial number, e.g. for [d435.yaml](https://github.com/intel/ros2_intel_realsense/blob/refactor/realsense_ros/config/d435.yaml#L3) in line3:
>serial_no: <serial_number_of_your_device> # d435


To start the camera node in ROS2, plug in the camera, then type the following command:

#### Single camera, taking d435 for example:

```bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/local_setup.bash

## using ros2 run
cd ~/ros2_ws
ros2 run realsense_node realsense_node __params:=`ros2 pkg prefix realsense_examples`/share/realsense_ros/config/d435.yaml __ns:=/d435
## using ros2 launch
ros2 launch realsense_examples rs_camera.launch.py
```

#### Multi camera, taking d435 and t265 for example:

* Terminal 1:

```bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/local_setup.bash

cd ~/ros2_ws
ros2 run realsense_node realsense_node __params:=`ros2 pkg prefix realsense_examples`/share/realsense_examples/config/d435.yaml __ns:=/d435
```

* Terminal 2:

```bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/local_setup.bash

cd ~/ros2_ws
ros2 run realsense_node realsense_node __params:=`ros2 pkg prefix realsense_examples`/share/realsense_examples/config/t265.yaml __ns:=/t265
```

* Or using ros2 launch :

```bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/local_setup.bash

cd ~/ros2_ws
## before launch all the cameras, serial number should be set in the launch file
ros2 launch realsense_examples rs_multiple_devices.launch.py
```

#### Multi camera with tf(transform), taking d435 and t265 for example:

```bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/local_setup.bash

cd ~/ros2_ws
## before launch all the cameras, serial number should be set in the launch file
ros2 launch realsense_examples rs_t265_and_d400.launch.py
```

***for more usage of these launch files in realsense_examples/launch, please refer to our [robot devkit](https://inte.github.io/robot_devkit) project, includes SLAM and navigation etc.***

### Configure Parameters at Runtime
Currently only support reconfigure parameters by `ros2 param` at runtime, e.g.  

Enable camera stream

```bash
ros2 param list
ros2 param get <node_name> color0.enabled
ros2 param set <node_name> color0.enabled true
```

Enable aligned depth

```bash
ros2 param set <node_name> align_depth true
```

Enable pointcloud

```bash
ros2 param set <node_name> enable_pointcloud true
```

## Published Topics

*  D435 and D415

[camera/color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/color/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/infra1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/infra1/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/infra2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/infra2/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/depth/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/depth/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/aligned_depth_to_color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/aligned_depth_to_color/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/pointcloud](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)

*  D435i  

[camera/color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/color/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/infra1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/infra1/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/infra2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/infra2/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/depth/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/depth/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/aligned_depth_to_color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/aligned_depth_to_color/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/pointcloud](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)

[camera/gyro/sample](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)

[camera/gyro/imu_info](https://github.com/intel/ros2_intel_realsense/blob/refactor/realsense_msgs/msg/IMUInfo.msg)

[camera/accel/sample](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)

[camera/accel/imu_info](https://github.com/intel/ros2_intel_realsense/blob/refactor/realsense_msgs/msg/IMUInfo.msg)

*  T265  

[camera/fisheye1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/fisheye1/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/fisheye2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[camera/fisheye1/camera_info](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg)

[camera/gyro/sample](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)

[camera/gyro/imu_info](https://github.com/intel/ros2_intel_realsense/blob/refactor/realsense_msgs/msg/IMUInfo.msg)

[camera/accel/sample](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg)

[camera/accel/imu_info](https://github.com/intel/ros2_intel_realsense/blob/refactor/realsense_msgs/msg/IMUInfo.msg)

[camera/odom/sample](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Odometry.msg)

## Known Issues

* Don't support dynamic reconfigure parameters by rqt_reconfigure due to [issue#53](https://github.com/ros-visualization/rqt_reconfigure/issues/53) of rqt_reconfigure, but you still can reconfigure parameters by `ros2 param`.
* We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit, but not support Mac OS X 10.12 (Sierra) and Windows 10 yet.

## TODO

* Add more tests
* Merge sensor data from gyroscope and accelerator into one topic
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

