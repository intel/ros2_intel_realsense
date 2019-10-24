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

#pragma once
#ifndef CAMERA_CORE_H  // NOLINT(build/header_guard)
#define CAMERA_CORE_H

#include <cstdlib>
#include <cctype>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "eigen3/Eigen/Geometry"
#include "librealsense2/rs.hpp"
#include "realsense_msgs/msg/imu_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "realsense/rs_constants.hpp"
#include "realsense/rs_base.hpp"
#include "cv_bridge/cv_bridge.h"

#include "image_transport/image_transport.h"
#include "image_transport/camera_subscriber.h"

const int STREAM_COUNT = 10;
const uint16_t RS435_RGB_PID = 0x0b07; // AWGC
const uint16_t RS435i_RGB_PID = 0x0B3A; // AWGC_MM
const uint16_t RS_T265_PID = 0x0b37; // T265
const double ROTATION_IDENTITY[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

int g_color_height_exp = 480;
int g_color_width_exp = 640;
int g_depth_height_exp = 480;
int g_depth_width_exp = 640;
int g_infrared1_height_exp = 480;
int g_infrared1_width_exp = 640;
int g_fisheye1_height_exp = 800;
int g_fisheye1_width_exp = 848;

uint32_t g_depth_step_exp;
uint32_t g_color_step_exp;
std::string g_depth_encoding_exp;
std::string g_color_encoding_exp;

bool g_enable_color = true;
bool g_enable_depth = true;
bool g_enable_infrared1 = true;
bool g_enable_infrared2 = true;
bool g_enable_pc = true;
bool g_enable_alignDepth = true;
bool g_enable_fisheye1 = true;
bool g_enable_fisheye2 = true;
bool g_enable_imu = true;

bool g_depth_recv = false;
bool g_color_recv = false;
bool g_infrared1_recv = false;
bool g_infrared2_recv = false;
bool g_pc_recv = false;
bool g_alignDepth_recv = false;
bool g_fisheye1_recv = false;
bool g_fisheye2_recv = false;
bool g_accel_recv = false;
bool g_gyro_recv = false;

float g_depth_avg = 0.0f;
float g_color_avg = 0.0f;
float g_infrared1_avg = 0.0f;
float g_infrared2_avg = 0.0f;
float g_pc_depth_avg = 0.0f;
float g_alignDepth_avg = 0.0f;
float g_fisheye1_avg = 0.0f;
float g_fisheye2_avg = 0.0f;

int g_height_recv[STREAM_COUNT] = {0};
int g_width_recv[STREAM_COUNT] = {0};
uint32_t g_step_recv[STREAM_COUNT] = {0};  // Received stream step.

std::string g_encoding_recv[STREAM_COUNT];  // Expected stream encoding.

int g_caminfo_height_recv[STREAM_COUNT] = {0};
int g_caminfo_width_recv[STREAM_COUNT] = {0};
double g_color_caminfo_D_recv[5] = {0.0};
double g_depth_caminfo_D_recv[5] = {0.0};
double g_infrared1_caminfo_D_recv[5] = {0.0};
double g_infrared2_caminfo_D_recv[5] = {0.0};
double g_alignDepth_caminfo_D_recv[5] = {0.0};
double g_fisheye1_caminfo_D_recv[5] = {0.0};
double g_fisheye2_caminfo_D_recv[5] = {0.0};

double g_caminfo_rotation_recv[STREAM_COUNT][9] = {{0.0}};
double g_caminfo_projection_recv[STREAM_COUNT][12] = {{0.0}};

std::string g_dmodel_recv[STREAM_COUNT];
std::string camera_type;


#endif  // CAMERA_CORE_H  // NOLINT(build/header_guard)
