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

#ifndef REALSENSE__RS_D435_HPP_
#define REALSENSE__RS_D435_HPP_

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "realsense/rs_base.hpp"
#include "realsense/rs_constants.hpp"

using namespace rclcpp::node_interfaces;

namespace realsense
{
class RealSenseD435 : public RealSenseBase
{
public:
  RealSenseD435(rs2::context ctx, rs2::device dev, rclcpp::Node & node);
  virtual ~RealSenseD435() = default;
  virtual void publishTopicsCallback(const rs2::frame & frame) override;
  virtual Result paramChangeCallback(const std::vector<rclcpp::Parameter> & params) override;
  void publishAlignedDepthTopic(const rs2::frame & frame, const rclcpp::Time & time);
  void publishSparsePointCloud(const rs2::points & points, const rs2::video_frame & color_frame, const rclcpp::Time & time);
  void publishDensePointCloud(const rs2::points & points, const rs2::video_frame & color_frame, const rclcpp::Time & time);
  void updateStreamCalibData(const rs2::video_stream_profile & video_profile);

protected:
  bool align_depth_;
  bool enable_pointcloud_;
  bool dense_pc_;
  bool initialized_ = false;
  rs2::align align_to_color_ = rs2::align(RS2_STREAM_COLOR);
  rs2::pointcloud pc_;
  rs2::points points_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aligned_depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

private:
  const std::vector<stream_index_pair> IMAGE_STREAMS = {COLOR, DEPTH, INFRA1, INFRA2};
};
}  // namespace realsense
#endif  // REALSENSE__RS_D435_HPP_
