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
  rclcpp::Time t = frameToTime(frame);
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
      } else if (param_name == "imu0.enabled") {
        result = toggleStream(IMU, param);
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

sensor_msgs::msg::Imu RealSenseD435I::CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data)
{
  sensor_msgs::msg::Imu imu_msg;
  rclcpp::Time t = msToTime(gyro_data.m_time);
  imu_msg.header.stamp = t;

  imu_msg.angular_velocity.x = gyro_data.m_data.x();
  imu_msg.angular_velocity.y = gyro_data.m_data.y();
  imu_msg.angular_velocity.z = gyro_data.m_data.z();

  imu_msg.linear_acceleration.x = accel_data.m_data.x();
  imu_msg.linear_acceleration.y = accel_data.m_data.y();
  imu_msg.linear_acceleration.z = accel_data.m_data.z();
  return imu_msg;
}

void RealSenseD435I::FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
  stream_index_pair type(imu_data.m_type);

  static CimuData _accel_data(ACCEL, {0,0,0}, -1.0);
  if (ACCEL == type)
  {
    _accel_data = imu_data;
    return;
  }
  if (_accel_data.m_time < 0)
    return;

  imu_msgs.push_back(CreateUnitedMessage(_accel_data, imu_data));
}

void RealSenseD435I::ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg)
{
//  TODO: get type_index properly
  auto type_index = GYRO;
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
}

template <typename T> T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

void RealSenseD435I::FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
  static std::deque<CimuData> _imu_history;
  _imu_history.push_back(imu_data);
  stream_index_pair type(imu_data.m_type);
  imu_msgs.clear();

  if ((type != ACCEL) || _imu_history.size() < 3)
    return;

  std::deque<CimuData> gyros_data;
  CimuData accel0, accel1, crnt_imu;

  while (_imu_history.size())
  {
    crnt_imu = _imu_history.front();
    _imu_history.pop_front();
    if (!accel0.is_set() && crnt_imu.m_type == ACCEL)
    {
      accel0 = crnt_imu;
    }
    else if (accel0.is_set() && crnt_imu.m_type == ACCEL)
    {
      accel1 = crnt_imu;
      const double dt = accel1.m_time - accel0.m_time;

      while (gyros_data.size())
      {
        CimuData crnt_gyro = gyros_data.front();
        gyros_data.pop_front();
        const double alpha = (crnt_gyro.m_time - accel0.m_time) / dt;
        CimuData crnt_accel(ACCEL, lerp(accel0.m_data, accel1.m_data, alpha), crnt_gyro.m_time);
        imu_msgs.push_back(CreateUnitedMessage(crnt_accel, crnt_gyro));
      }
      accel0 = accel1;
    }
    else if (accel0.is_set() && crnt_imu.m_time >= accel0.m_time && crnt_imu.m_type == GYRO)
    {
      gyros_data.push_back(crnt_imu);
    }
  }
  _imu_history.push_back(crnt_imu);
  return;
}

}  // namespace realsense
