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

  auto depth_sensor =  dev.first<rs2::depth_sensor>();
  if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
  {
//    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
  }

  std::string unite_imu_method_str("");
  node_.get_parameter_or("unite_imu_method", unite_imu_method_str, DEFAULT_UNITE_IMU_METHOD);
  if (unite_imu_method_str == "linear_interpolation")
    imu_sync_method_ = imu_sync_method::LINEAR_INTERPOLATION;
  else if (unite_imu_method_str == "copy")
    imu_sync_method_ = imu_sync_method::COPY;
  else {
    RCLCPP_ERROR(node_.get_logger(), "Invalid value for unite_imu_method: %s", unite_imu_method_str.c_str());
  }
}

void RealSenseD435I::publishTopicsCallback(const rs2::frame & frame)
{
  if (frame.is<rs2::frameset>()) {
    RealSenseD435::publishTopicsCallback(frame);
  } else if (frame.is<rs2::motion_frame>()) {
    if ((enable_[ACCEL] && (imu_pub_[ACCEL]->get_subscription_count() > 0 || imu_info_pub_[ACCEL]->get_subscription_count() > 0))
      || (enable_[GYRO] && (imu_pub_[GYRO]->get_subscription_count() > 0 || imu_info_pub_[GYRO]->get_subscription_count() > 0))
      || (enable_[IMU] && (imu_pub_[IMU]->get_subscription_count() > 0 || imu_info_pub_[IMU]->get_subscription_count() > 0))) {
      publishIMUTopic(frame);
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

void RealSenseD435I::publishIMUTopic(const rs2::frame & frame)
{
  auto type = frame.get_profile().stream_type();
  auto index = frame.get_profile().stream_index();
  auto type_index = std::pair<rs2_stream, int>(type, index);
  auto m_frame = frame.as<rs2::motion_frame>();
  sensor_msgs::msg::Imu imu_msg;
  realsense_msgs::msg::IMUInfo info_msg;

  imu_msg.header.frame_id = OPTICAL_FRAME_ID.at(type_index);
  imu_msg.header.stamp = frameToTime(frame);
  ImuMessage_AddDefaultValues(imu_msg);

  auto imu_data = m_frame.get_motion_data();
  if (type_index == GYRO) {
    imu_msg.angular_velocity.x = imu_data.x;
    imu_msg.angular_velocity.y = imu_data.y;
    imu_msg.angular_velocity.z = imu_data.z;
  } else if (type_index == ACCEL) {
    imu_msg.linear_acceleration.x = imu_data.x;
    imu_msg.linear_acceleration.y = imu_data.y;
    imu_msg.linear_acceleration.z = imu_data.z;
  }

  if (imu_pub_[type_index]->get_subscription_count() > 0){
    imu_pub_[type_index]->publish(imu_msg);
  }

  if (imu_info_pub_[type_index]->get_subscription_count() > 0){
    info_msg = getIMUInfo(frame, type_index);
    imu_info_pub_[type_index]->publish(info_msg);
  }

  if (imu_pub_[IMU]->get_subscription_count() > 0 || imu_info_pub_[IMU]->get_subscription_count() > 0) {
    imu_callback_sync(frame, imu_sync_method_);
  }
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
  imu_msg.header.stamp = msToTime(gyro_data.m_time);

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

void RealSenseD435I::imu_callback_sync(const rs2::frame & frame, imu_sync_method sync_method)
{
  static std::mutex m_mutex;

  m_mutex.lock();

  auto type = frame.get_profile().stream_type();
  auto index = frame.get_profile().stream_index();
  auto type_index = std::pair<rs2_stream, int>(type, index);

  double frame_time = frame.get_timestamp();

//  TODO: Sort out initialized_time with ros2 node clock
//  bool placeholder_false(false);
//  if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
//  {
//    setBaseTime(frame_time, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain());
//  }
//  double elapsed_camera_ms = (/*ms*/ frame_time - /*ms*/ _camera_time_base) / 1000.0;
  double elapsed_camera_ms = frame_time;

//  if (0 != _synced_imu_publisher->getNumSubscribers())
  if (true)
  {
    auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
    Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
    CimuData imu_data(type_index, v, elapsed_camera_ms);
    std::deque<sensor_msgs::msg::Imu> imu_msgs;
    switch (sync_method)
    {
      case COPY:
        FillImuData_Copy(imu_data, imu_msgs);
        break;
      case LINEAR_INTERPOLATION:
        FillImuData_LinearInterpolation(imu_data, imu_msgs);
        break;
    }
    while (imu_msgs.size())
    {
      auto imu_msg = imu_msgs.front();

//      TODO: Sort out _ros_time_base with ros2 node clock
//      ros::Time t(_ros_time_base.toSec() + imu_msg.header.stamp.toSec());

//      rclcpp::Time t = frameToTime(frame);
//      imu_msg.header.stamp = t;

      imu_msg.header.frame_id = OPTICAL_FRAME_ID.at(IMU);
      ImuMessage_AddDefaultValues(imu_msg);
      imu_pub_[IMU]->publish(imu_msg);

//      info_msg = getIMUInfo(frame, IMU);
//      imu_info_pub_[IMU]->publish(info_msg);
      RCLCPP_DEBUG(node_.get_logger(), "Publish united %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
      imu_msgs.pop_front();
    }
  }
  m_mutex.unlock();
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
