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

#include <string>

#pragma once
#ifndef REALSENSE_ROS2_CAMERA__CONSTANTS_HPP_
#define REALSENSE_ROS2_CAMERA__CONSTANTS_HPP_

#define REALSENSE_ROS_MAJOR_VERSION    2
#define REALSENSE_ROS_MINOR_VERSION    0
#define REALSENSE_ROS_PATCH_VERSION    1

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
/* Return version in "X.Y.Z" format */
#define REALSENSE_ROS_VERSION_STR (VAR_ARG_STRING(REALSENSE_ROS_MAJOR_VERSION. \
  REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))

namespace realsense_ros2_camera
{
const bool POINTCLOUD = true;
const bool ALIGN_POINTCLOUD = false;
const bool SYNC_FRAMES = true;

const bool ALIGN_DEPTH = true;

const int DEPTH_WIDTH = 640;
const int DEPTH_HEIGHT = 480;

const int INFRA1_WIDTH = 640;
const int INFRA1_HEIGHT = 480;

const int INFRA2_WIDTH = 640;
const int INFRA2_HEIGHT = 480;

const int COLOR_WIDTH = 640;
const int COLOR_HEIGHT = 480;

const int FISHEYE_WIDTH = 640;
const int FISHEYE_HEIGHT = 480;


const int DEPTH_FPS = 30;
const int INFRA1_FPS = 30;
const int INFRA2_FPS = 30;
const int COLOR_FPS = 30;
const int FISHEYE_FPS = 30;
const int GYRO_FPS = 1000;
const int ACCEL_FPS = 1000;


const bool ENABLE_DEPTH = true;
const bool ENABLE_INFRA1 = true;
const bool ENABLE_INFRA2 = true;
const bool ENABLE_COLOR = true;
const bool ENABLE_FISHEYE = true;
const bool ENABLE_IMU = true;


const char DEFAULT_BASE_FRAME_ID[] = "camera_link";
const char DEFAULT_DEPTH_FRAME_ID[] = "camera_depth_frame";
const char DEFAULT_INFRA1_FRAME_ID[] = "camera_infra1_frame";
const char DEFAULT_INFRA2_FRAME_ID[] = "camera_infra2_frame";
const char DEFAULT_COLOR_FRAME_ID[] = "camera_color_frame";
const char DEFAULT_FISHEYE_FRAME_ID[] = "camera_fisheye_frame";
const char DEFAULT_IMU_FRAME_ID[] = "camera_imu_frame";
const char DEFAULT_DEPTH_OPTICAL_FRAME_ID[] = "camera_depth_optical_frame";
const char DEFAULT_INFRA1_OPTICAL_FRAME_ID[] = "camera_infra1_optical_frame";
const char DEFAULT_INFRA2_OPTICAL_FRAME_ID[] = "camera_infra2_optical_frame";
const char DEFAULT_COLOR_OPTICAL_FRAME_ID[] = "camera_color_optical_frame";
const char DEFAULT_FISHEYE_OPTICAL_FRAME_ID[] = "camera_fisheye_optical_frame";
const char DEFAULT_ACCEL_OPTICAL_FRAME_ID[] = "camera_accel_optical_frame";
const char DEFAULT_GYRO_OPTICAL_FRAME_ID[] = "camera_gyro_optical_frame";
const char DEFAULT_IMU_OPTICAL_FRAME_ID[] = "camera_imu_optical_frame";
}  // namespace realsense_ros2_camera
#endif  // REALSENSE_ROS2_CAMERA__CONSTANTS_HPP_
