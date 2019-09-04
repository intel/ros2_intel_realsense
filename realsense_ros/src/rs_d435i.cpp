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

#include "realsense/rs_d435i.hpp"

namespace realsense
{
RealSenseD435I::RealSenseD435I(rs2::context ctx, rs2::device dev, rclcpp::Node & node)
: RealSenseD435(ctx, dev, node)
{ 
  for (auto & stream : MOTION_STREAMS) {
    setupStream(stream);
  }
  // if (enable_[ACCEL] == true) {
  //   linear_accel_cov_ = node_.declare_parameter("accel0.linear_acceleration_covariance", DEFAULT_LINEAR_ACCEL_COV);
  // }
  // if (enable_[GYRO] == true) {
  //   angular_velocity_cov_ = node_.declare_parameter("gyro0.angular_velocity_covariance", DEFAULT_ANGULAR_VELOCITY_COV);
  // }
  linear_accel_cov_ = DEFAULT_LINEAR_ACCEL_COV;
  angular_velocity_cov_ = DEFAULT_ANGULAR_VELOCITY_COV;
  initialized_ = true;
}

void RealSenseD435I::publishTopicsCallback(const rs2::frame & frame)
{
  rclcpp::Time t = node_.now();
  if (frame.is<rs2::frameset>()) {
    RealSenseD435::publishTopicsCallback(frame);
  } else if (frame.is<rs2::motion_frame>()) {
    if ((enable_[ACCEL] && (imu_pub_[ACCEL]->get_subscription_count() > 0 || imu_info_pub_[ACCEL]->get_subscription_count() > 0))
      || (enable_[GYRO] && (imu_pub_[GYRO]->get_subscription_count() > 0 || imu_info_pub_[GYRO]->get_subscription_count() > 0))) {
      publishIMUTopic(frame, t);
    }
  }
}

Result RealSenseD435I::paramChangeCallback(const std::vector<rclcpp::Parameter> & params)
{
  auto result = Result();
  result.successful = true;
  if (this->initialized_ == true) {
    result = RealSenseD435::paramChangeCallback(params);
    for (auto & param : params) {
      auto param_name = param.get_name();
      if (param_name == "accel0.enabled") {
        result = toggleStream(ACCEL, param);
      } else if (param_name == "gyro0.enabled") {
        result = toggleStream(GYRO, param);
      }
    }
  }
  return result;
}

void RealSenseD435I::publishIMUTopic(const rs2::frame & frame, const rclcpp::Time & time)
{
  auto type = frame.get_profile().stream_type();
  auto index = frame.get_profile().stream_index();
  auto type_index = std::pair<rs2_stream, int>(type, index);
  auto m_frame = frame.as<rs2::motion_frame>();
  sensor_msgs::msg::Imu imu_msg;
  realsense_msgs::msg::IMUInfo info_msg;

  imu_msg.header.frame_id = OPTICAL_FRAME_ID.at(type_index);
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;
  imu_msg.orientation_covariance = {-1.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0};
  imu_msg.linear_acceleration_covariance = {linear_accel_cov_, 0.0, 0.0,
                                            0.0, linear_accel_cov_, 0.0,
                                            0.0, 0.0, linear_accel_cov_};
  imu_msg.angular_velocity_covariance = {angular_velocity_cov_, 0.0, 0.0,
                                         0.0, angular_velocity_cov_, 0.0,
                                         0.0, 0.0, angular_velocity_cov_};

  auto imu_data = m_frame.get_motion_data();
  if (type_index == GYRO) {
    imu_msg.angular_velocity.x = imu_data.x;
    imu_msg.angular_velocity.y = imu_data.y;
    imu_msg.angular_velocity.z = imu_data.z;
    info_msg = getIMUInfo(frame, GYRO);
  } else if (type_index == ACCEL) {
    imu_msg.linear_acceleration.x = imu_data.x;
    imu_msg.linear_acceleration.y = imu_data.y;
    imu_msg.linear_acceleration.z = imu_data.z;
    info_msg = getIMUInfo(frame, ACCEL);
  }
  imu_msg.header.stamp = time;
  imu_pub_[type_index]->publish(imu_msg);
  imu_info_pub_[type_index]->publish(info_msg);
}

IMUInfo RealSenseD435I::getIMUInfo(const rs2::frame & frame, const stream_index_pair & stream_index)
{
  auto m_profile = frame.get_profile().as<rs2::motion_stream_profile>();
  realsense_msgs::msg::IMUInfo info;
  rs2_motion_device_intrinsic imu_intrinsics;
  try {
    imu_intrinsics = m_profile.get_motion_intrinsics();
  } catch (const std::runtime_error &ex) {
    RCLCPP_INFO(node_.get_logger(), "No Motion Intrinsics available. Please calibrate it by rs-imu-calibration tool first.");
    imu_intrinsics = {{{1,0,0,0},
                       {0,1,0,0},
                       {0,0,1,0}}, {0,0,0}, {0,0,0}};
  }

  auto index = 0;
  info.header.frame_id = OPTICAL_FRAME_ID.at(stream_index);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      info.data[index] = imu_intrinsics.data[i][j];
      ++index;
    }
    info.noise_variances[i] =  imu_intrinsics.noise_variances[i];
    info.bias_variances[i] = imu_intrinsics.bias_variances[i];
  }
  return info;
}
}  // namespace realsense
