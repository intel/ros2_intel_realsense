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
  void publishIMUTopic(const rs2::frame & frame, const rclcpp::Time & time);
  IMUInfo getIMUInfo(const rs2::frame & frame, const stream_index_pair & stream_index);

private:
  const std::vector<stream_index_pair> IMAGE_STREAMS = {COLOR, DEPTH, INFRA1, INFRA2};
  const std::vector<stream_index_pair> MOTION_STREAMS = {ACCEL, GYRO};
  double linear_accel_cov_;
  double angular_velocity_cov_;
  bool initialized_ = false;
};
}  // namespace perception
#endif // REALSENSE__RS_D435I_HPP_