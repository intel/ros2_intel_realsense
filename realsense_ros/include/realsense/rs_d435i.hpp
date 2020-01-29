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

#ifndef REALSENSE__RS_D435I_HPP_
#define REALSENSE__RS_D435I_HPP_

#include "realsense/rs_d435.hpp"

#include <queue>

using IMUInfo = realsense_msgs::msg::IMUInfo;

namespace realsense
{
class RealSenseD435I : public RealSenseD435
{
public:
  RealSenseD435I(rs2::context ctx, rs2::device dev, rclcpp::Node & node);
  virtual ~RealSenseD435I() = default;
  virtual void publishTopicsCallback(const rs2::frame & frame) override;
  virtual Result paramChangeCallback(const std::vector<rclcpp::Parameter> & params) override;
  void publishIMUTopic(const rs2::frame & frame);
  IMUInfo getIMUInfo(const rs2::frame & frame, const stream_index_pair & stream_index);

 public:
  enum imu_sync_method{COPY, LINEAR_INTERPOLATION};

 protected:
  class float3
  {
   public:
    float x, y, z;

   public:
    float3& operator*=(const float& factor)
    {
      x*=factor;
      y*=factor;
      z*=factor;
      return (*this);
    }
    float3& operator+=(const float3& other)
    {
      x+=other.x;
      y+=other.y;
      z+=other.z;
      return (*this);
    }
  };

 private:
  class CimuData
  {
   public:
    CimuData() : m_time(-1) {};
    CimuData(const stream_index_pair type, Eigen::Vector3d data, double time):
        m_type(type),
        m_data(data),
        m_time(time){};
    bool is_set() {return m_time > 0;};
   public:
    stream_index_pair m_type;
    Eigen::Vector3d m_data;
    double          m_time;
  };

private:
  sensor_msgs::msg::Imu CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

  void FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
  void ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu &imu_msg);
  void imu_callback_sync(const rs2::frame & frame, imu_sync_method sync_method);
  void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);

private:
  const std::vector<stream_index_pair> IMAGE_STREAMS = {COLOR, DEPTH, INFRA1, INFRA2};
  const std::vector<stream_index_pair> MOTION_STREAMS = {ACCEL, GYRO, IMU};
  double linear_accel_cov_;
  double angular_velocity_cov_;
  bool initialized_ = false;
  imu_sync_method imu_sync_method_;
};
}  // namespace perception
#endif // REALSENSE__RS_D435I_HPP_