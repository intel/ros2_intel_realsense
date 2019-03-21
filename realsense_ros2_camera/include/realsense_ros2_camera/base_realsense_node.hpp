// Copyright (c) 2018 Intel Corporation. All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REALSENSE_ROS2_CAMERA__BASE_REALSENSE_NODE_HPP_
#define REALSENSE_ROS2_CAMERA__BASE_REALSENSE_NODE_HPP_

// cpplint: c system headers
#include <eigen3/Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <console_bridge/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
// cpplint: c++ system headers
#include <algorithm>
#include <csignal>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
// cpplint: other headers
#include "realsense_ros2_camera/constants.hpp"
#include "realsense_camera_msgs/msg/imu_info.hpp"
#include "realsense_camera_msgs/msg/extrinsics.hpp"

namespace realsense_ros2_camera
{
using realsense_camera_msgs::msg::Extrinsics;
using realsense_camera_msgs::msg::IMUInfo;
using stream_index_pair = std::pair<rs2_stream, int>;

const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    
const std::vector<std::vector<stream_index_pair>> IMAGE_STREAMS = {{{DEPTH, INFRA1, INFRA2},
    {COLOR}, {FISHEYE}}};

const std::vector<std::vector<stream_index_pair>> HID_STREAMS = {{GYRO, ACCEL}};

class PipelineSyncer : public rs2::asynchronous_syncer
{
public:
  void operator()(rs2::frame f) const
  {
    invoke(std::move(f));
  }
};

class RealSenseCameraNode : public rclcpp::Node
{
public:
  RealSenseCameraNode();
  virtual ~RealSenseCameraNode() = default;
  virtual void onInit();
private:
  void getParameters();
  void setupDevice();
  void setupPublishers();
  void setupStreams();
  void updateStreamCalibData(const rs2::video_stream_profile & video_profile);
  Eigen::Quaternionf rotationMatrixToQuaternion(float rotation[9]) const;
  void publishStaticTransforms();
//   void alignFrame(const rs2_intrinsics & from_intrin,
//                   const rs2_intrinsics & other_intrin,
//                   rs2::frame from_image,
//                   uint32_t output_image_bytes_per_pixel,
//                   const rs2_extrinsics & from_to_other,
//                   std::vector<uint8_t> & out_vec);
//   rs2_extrinsics getRsExtrinsics(const stream_index_pair & from_stream,
//                                  const stream_index_pair & to_stream);
  void publishFrame(rs2::frame f, const rclcpp::Time & t);
  void publishAlignedDepthImg(rs2::frame depth_frame, const rclcpp::Time & t);
  void publishPCTopic(const rclcpp::Time & t);
  void publishAlignedPCTopic(const rclcpp::Time & t);
  Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics & extrinsics) const;
  Extrinsics getFisheye2ImuExtrinsicsMsg();
  Extrinsics getFisheye2DepthExtrinsicsMsg();
  IMUInfo getImuInfo(const stream_index_pair & stream_index);
  void tryGetLogSeverity(rs2_log_severity & severity) const;
  bool getEnabledProfile(const stream_index_pair & stream_index, rs2::stream_profile & profile);
  
  struct float3
  {
    float x, y, z;
  };
  
  std::unique_ptr<rs2::context> ctx_;
  std::string serial_no_;
  float depth_scale_meters_;
  
  std::map<stream_index_pair, std::unique_ptr<rs2::sensor>> sensors_;
  std::map<stream_index_pair, rs2_intrinsics> stream_intrinsics_;
  std::map<stream_index_pair, int> width_;
  std::map<stream_index_pair, int> height_;
  std::map<stream_index_pair, int> fps_;
  std::map<stream_index_pair, bool> enable_;
  
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  std::map<stream_index_pair, image_transport::Publisher> image_publishers_;
  std::map<stream_index_pair,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> info_publisher_;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_publishers_;
  std::map<stream_index_pair,
    rclcpp::Publisher<realsense_camera_msgs::msg::IMUInfo>::SharedPtr> imu_info_publisher_;
  rclcpp::Publisher<realsense_camera_msgs::msg::Extrinsics>::SharedPtr fe_to_depth_publisher_,
    fe_to_imu_publisher_;
    
  image_transport::Publisher align_depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr align_depth_camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr align_pointcloud_publisher_;
  
  std::map<stream_index_pair, rs2_format> format_;
  std::map<stream_index_pair, int> image_format_;
  std::map<stream_index_pair, std::string> encoding_;
  std::map<stream_index_pair, int> unit_step_size_;
  std::map<stream_index_pair, std::string> stream_name_;
  
  std::map<stream_index_pair, cv::Mat> image_;
  std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> camera_info_;
  std::map<stream_index_pair, std::vector<rs2::stream_profile>> enabled_profiles_;
  
  std::string base_frame_id_;
  std::map<stream_index_pair, std::string> frame_id_;
  std::map<stream_index_pair, std::string> optical_frame_id_;
  std::map<stream_index_pair, int> seq_;
  
  bool intialize_time_base_;
  double camera_time_base_;
  
  rclcpp::Clock ros_clock_;
  rclcpp::Time ros_time_base_;
  rclcpp::Logger logger_;
  bool sync_frames_;
  bool pointcloud_;
  bool align_pointcloud_;
  bool align_depth_;
  PipelineSyncer syncer_;
  rs2_extrinsics depth2color_extrinsics_;
  rs2::frameset aligned_frameset_;
};
}  // namespace realsense_ros2_camera
#endif  // REALSENSE_ROS2_CAMERA__BASE_REALSENSE_NODE_HPP_
