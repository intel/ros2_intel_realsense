// Copyright (c) 2019 Intel Corporation. All Rights Reserved
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

#ifndef REALSENSE__RS_CONSTANTS_HPP_
#define REALSENSE__RS_CONSTANTS_HPP_

#include "cv_bridge/cv_bridge.h"

namespace realsense
{
  const uint16_t RS415_PID        = 0x0ad3; // ASRC
  const uint16_t RS435_RGB_PID    = 0x0b07; // AWGC
  const uint16_t RS435i_RGB_PID   = 0x0B3A; // AWGC_MM
  const uint16_t RS_T265_PID      = 0x0b37; // T265

  using stream_index_pair = std::pair<rs2_stream, int>;

  const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
  const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
  const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
  const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
  const stream_index_pair FISHEYE1{RS2_STREAM_FISHEYE, 1};
  const stream_index_pair FISHEYE2{RS2_STREAM_FISHEYE, 2};
  const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
  const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
  const stream_index_pair POSE{RS2_STREAM_POSE, 0};

  const bool ALIGN_DEPTH = true;
  const bool ENABLE_POINTCLOUD = true;
  const bool DENSE_PC = true;
  const bool DEFAULT_ENABLE_STREAM = true;

  const std::vector<int> DEFAULT_IMAGE_RESOLUTION = {640, 480};
  const std::vector<int> FISHEYE_RESOLUTION = {848, 800};
  const int DEFAULT_IMAGE_FPS = 30;

  const double DEFAULT_LINEAR_ACCEL_COV = 0.01;
  const double DEFAULT_ANGULAR_VELOCITY_COV = 0.01;

  const std::string DEFAULT_BASE_FRAME_ID             = "camera_link";
  const std::string DEFAULT_ODOM_FRAME_ID             = "odom_frame";
  const std::string DEFAULT_DEPTH_FRAME_ID            = "camera_depth_frame";
  const std::string DEFAULT_INFRA1_FRAME_ID           = "camera_infra1_frame";
  const std::string DEFAULT_INFRA2_FRAME_ID           = "camera_infra2_frame";
  const std::string DEFAULT_COLOR_FRAME_ID            = "camera_color_frame";
  const std::string DEFAULT_FISHEYE1_FRAME_ID         = "camera_fisheye1_frame";
  const std::string DEFAULT_FISHEYE2_FRAME_ID         = "camera_fisheye2_frame";
  const std::string DEFAULT_POSE_FRAME_ID             = "camera_pose_frame";

  const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID    = "camera_depth_optical_frame";
  const std::string DEFAULT_INFRA1_OPTICAL_FRAME_ID   = "camera_infra1_optical_frame";
  const std::string DEFAULT_INFRA2_OPTICAL_FRAME_ID   = "camera_infra2_optical_frame";
  const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID    = "camera_color_optical_frame";
  const std::string DEFAULT_FISHEYE1_OPTICAL_FRAME_ID = "camera_fisheye1_optical_frame";
  const std::string DEFAULT_FISHEYE2_OPTICAL_FRAME_ID = "camera_fisheye2_optical_frame";
  const std::string DEFAULT_ACCEL_OPTICAL_FRAME_ID    = "camera_accel_optical_frame";
  const std::string DEFAULT_GYRO_OPTICAL_FRAME_ID     = "camera_gyro_optical_frame";
  const std::string DEFAULT_POSE_OPTICAL_FRAME_ID     = "camera_pose_optical_frame";

  const std::string DEFAULT_ALIGNED_DEPTH_TO_COLOR_FRAME_ID = "camera_aligned_depth_to_color_frame";

  const std::string DEFAULT_UNITE_IMU_METHOD         = "";
  const std::string DEFAULT_FILTERS                  = "";
  const std::string DEFAULT_TOPIC_ODOM_IN            = "";

  const std::map<rs2_stream, int> CV_FORMAT = {{RS2_STREAM_COLOR, CV_8UC3},
                                               {RS2_STREAM_DEPTH, CV_16UC1},
                                               {RS2_STREAM_INFRARED, CV_8UC1},
                                               {RS2_STREAM_FISHEYE, CV_8UC1}};

  const std::map<rs2_stream, std::string> MSG_ENCODING = {{RS2_STREAM_COLOR, sensor_msgs::image_encodings::RGB8},
                                                          {RS2_STREAM_DEPTH, sensor_msgs::image_encodings::TYPE_16UC1},
                                                          {RS2_STREAM_INFRARED, sensor_msgs::image_encodings::TYPE_8UC1},
                                                          {RS2_STREAM_FISHEYE, sensor_msgs::image_encodings::TYPE_8UC1}};

  const std::map<rs2_stream, rs2_format> STREAM_FORMAT = {{RS2_STREAM_COLOR, RS2_FORMAT_RGB8},
                                                          {RS2_STREAM_DEPTH, RS2_FORMAT_Z16},
                                                          {RS2_STREAM_INFRARED, RS2_FORMAT_Y8},
                                                          {RS2_STREAM_FISHEYE, RS2_FORMAT_Y8},
                                                          {RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F},
                                                          {RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F},
                                                          {RS2_STREAM_POSE, RS2_FORMAT_6DOF}};

  const std::map<rs2_stream, std::string> STREAM_NAME = {{RS2_STREAM_COLOR, "color"},
                                                         {RS2_STREAM_DEPTH, "depth"},
                                                         {RS2_STREAM_INFRARED, "infra"},
                                                         {RS2_STREAM_ACCEL, "accel"},
                                                         {RS2_STREAM_GYRO, "gyro"},
                                                         {RS2_STREAM_FISHEYE, "fisheye"},
                                                         {RS2_STREAM_POSE, "pose"}};

  const std::map<stream_index_pair, std::string> OPTICAL_FRAME_ID = {{COLOR, DEFAULT_COLOR_OPTICAL_FRAME_ID},
                                                                     {DEPTH, DEFAULT_DEPTH_OPTICAL_FRAME_ID},
                                                                     {INFRA1, DEFAULT_INFRA1_OPTICAL_FRAME_ID},
                                                                     {INFRA2, DEFAULT_INFRA2_OPTICAL_FRAME_ID},
                                                                     {FISHEYE1, DEFAULT_FISHEYE1_OPTICAL_FRAME_ID},
                                                                     {FISHEYE2, DEFAULT_FISHEYE2_OPTICAL_FRAME_ID},
                                                                     {ACCEL, DEFAULT_ACCEL_OPTICAL_FRAME_ID},
                                                                     {GYRO, DEFAULT_GYRO_OPTICAL_FRAME_ID},
                                                                     {POSE, DEFAULT_POSE_OPTICAL_FRAME_ID}};

  const std::map<stream_index_pair, std::string> SAMPLE_TOPIC = {{COLOR, "camera/color/image_raw"},
                                                                 {DEPTH, "camera/depth/image_rect_raw"},
                                                                 {INFRA1, "camera/infra1/image_rect_raw"},
                                                                 {INFRA2, "camera/infra2/image_rect_raw"},
                                                                 {FISHEYE1, "camera/fisheye1/image_raw"},
                                                                 {FISHEYE2, "camera/fisheye2/image_raw"},
                                                                 {ACCEL, "camera/accel/sample"},
                                                                 {GYRO, "camera/gyro/sample"},
                                                                 {POSE, "camera/odom/sample"}};

  const std::map<stream_index_pair, std::string> INFO_TOPIC = {{COLOR, "camera/color/camera_info"},
                                                               {DEPTH, "camera/depth/camera_info"},
                                                               {INFRA1, "camera/infra1/camera_info"},
                                                               {INFRA2, "camera/infra2/camera_info"},
                                                               {FISHEYE1, "camera/fisheye1/camera_info"},
                                                               {FISHEYE2, "camera/fisheye2/camera_info"},
                                                               {ACCEL, "camera/accel/imu_info"},
                                                               {GYRO, "camera/gyro/imu_info"},
                                                               {POSE, ""}};

  const std::string ALIGNED_DEPTH_IMAGE_TOPIC = "camera/aligned_depth_to_color/image_raw";
  const std::string ALIGNED_DEPTH_INFO_TOPIC = "camera/aligned_depth_to_color/camera_info";
  const std::string POINTCLOUD_TOPIC = "camera/pointcloud";
}  // namespace realsense

#endif  // REALSENSE__RS_CONSTANTS_HPP_
