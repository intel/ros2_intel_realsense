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

#include <opencv2/opencv.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "realsense/rs_d435.hpp"

namespace realsense
{
RealSenseD435::RealSenseD435(rs2::context ctx, rs2::device dev, rclcpp::Node & node)
: RealSenseBase(ctx, dev, node)
{ 
  for (auto & stream : IMAGE_STREAMS) {
    setupStream(stream);
  }

  if (node_.has_parameter("align_depth")) {
    node_.get_parameter("align_depth", align_depth_);
  } else {
    align_depth_ = node_.declare_parameter("align_depth", ALIGN_DEPTH);
  }
  if (align_depth_ == true) {
    if (enable_[COLOR] == false || enable_[DEPTH] == false) {
      RCLCPP_WARN(node_.get_logger(), "Make sure color and depth frame are enabled before aligning depth to color.");
    }
  }
  aligned_depth_image_pub_ = node_.create_publisher<sensor_msgs::msg::Image>(ALIGNED_DEPTH_IMAGE_TOPIC, rclcpp::QoS(1));
  aligned_depth_info_pub_ = node_.create_publisher<sensor_msgs::msg::CameraInfo>(ALIGNED_DEPTH_INFO_TOPIC, rclcpp::QoS(1));

  if (node_.has_parameter("enable_pointcloud")) {
    node_.get_parameter("enable_pointcloud", enable_pointcloud_);
  } else {
    enable_pointcloud_ = node_.declare_parameter("enable_pointcloud", ENABLE_POINTCLOUD);
  }
  if (enable_pointcloud_ == true) {
    if (enable_[COLOR] == false || enable_[DEPTH] == false) {
      RCLCPP_WARN(node_.get_logger(), "Make sure color and depth frame are enabled before publishing pointcloud.");
    }
  }
  pointcloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(POINTCLOUD_TOPIC, rclcpp::QoS(1));
  if (node_.has_parameter("dense_pointcloud")) {
    node_.get_parameter("dense_pointcloud", dense_pc_);
  } else {
    dense_pc_ = node_.declare_parameter("dense_pointcloud", DENSE_PC);
  }
  initialized_ = true;
}

void RealSenseD435::publishTopicsCallback(const rs2::frame & frame)
{
  rs2::frameset frameset = frame.as<rs2::frameset>();
  rclcpp::Time t = node_.now();
  if (enable_[COLOR] && (image_pub_[COLOR]->get_subscription_count() > 0 || camera_info_pub_[COLOR]->get_subscription_count() > 0)){
    auto frame = frameset.get_color_frame();
    publishImageTopic(frame, t);
  }
  if (enable_[DEPTH] && (image_pub_[DEPTH]->get_subscription_count() > 0 || camera_info_pub_[DEPTH]->get_subscription_count() > 0)) {
    auto frame = frameset.get_depth_frame();
    publishImageTopic(frame, t);
  }
  if (enable_[INFRA1] && (image_pub_[INFRA1]->get_subscription_count() > 0 || camera_info_pub_[INFRA1]->get_subscription_count() > 0)) {
    auto frame = frameset.get_infrared_frame(1);
    publishImageTopic(frame, t);
  }
  if (enable_[INFRA2] && (image_pub_[INFRA2]->get_subscription_count() > 0 || camera_info_pub_[INFRA2]->get_subscription_count() > 0)) {
    auto frame = frameset.get_infrared_frame(2);
    publishImageTopic(frame, t);
  }
   if ((enable_pointcloud_ && pointcloud_pub_->get_subscription_count() > 0) || 
      (align_depth_ && (aligned_depth_image_pub_->get_subscription_count() > 0 || 
       aligned_depth_info_pub_->get_subscription_count() > 0))) {
    auto aligned_frameset = align_to_color_.process(frameset);
    auto depth = aligned_frameset.get_depth_frame();
    if (aligned_depth_image_pub_->get_subscription_count() > 0 || aligned_depth_info_pub_->get_subscription_count() > 0) {
      publishAlignedDepthTopic(depth, t);
    }
    if (pointcloud_pub_->get_subscription_count() > 0) {
      auto color_frame = frameset.get_color_frame();
      points_ = pc_.calculate(depth);
      if (dense_pc_ == true) {
        publishDensePointCloud(points_, color_frame, t);
      } else {
        publishSparsePointCloud(points_, color_frame, t);
      }
    }
  }
}

Result RealSenseD435::paramChangeCallback(const std::vector<rclcpp::Parameter> & params)
{
  auto result = Result();
  result.successful = true;
  if (initialized_ == true) {
    for (auto & param : params) {
      auto param_name = param.get_name();
      if (param_name == "color0.enabled") {
        result = toggleStream(COLOR, param);
      } else if (param_name == "color0.resolution") {
        result = changeResolution(COLOR, param);
      } else if (param_name == "color0.fps") {
        result = changeFPS(COLOR, param);
      } else if (param_name == "depth0.enabled") {
        result = toggleStream(DEPTH, param);
      } else if (param_name == "depth0.resolution") {
        result = changeResolution(DEPTH, param);
      } else if (param_name == "depth0.fps") {
        result = changeFPS(DEPTH, param);
      } else if (param_name == "infra1.enabled") {
        result = toggleStream(INFRA1, param);
      } else if (param_name == "infra1.resolution") {
        result = changeResolution(INFRA1, param);
      } else if (param_name == "infra1.fps") {
        result = changeFPS(INFRA1, param);
      } else if (param_name == "infra2.enabled") {
        result = toggleStream(INFRA2, param);
      } else if (param_name == "infra2.resolution") {
        result = changeResolution(INFRA2, param);
      } else if (param_name == "infra2.fps") {
        result = changeFPS(INFRA2, param);
      } else if (param_name == "align_depth") {
        auto param_value = param.as_bool();
        if (param_value != align_depth_) {
          align_depth_ = param_value;
        } else {
          result.successful = false;
          result.reason = "Parameter is equal to the previous value. Do nothing.";
        }
      } else if (param_name == "enable_pointcloud") {
        auto param_value = param.as_bool();
        if (param_value != enable_pointcloud_) {
          enable_pointcloud_ = param_value;
        } else {
          result.successful = false;
          result.reason = "Parameter is equal to the previous value. Do nothing.";
        }
      } else if (param_name == "dense_pointcloud") {
        auto param_value = param.as_bool();
        if (param_value != dense_pc_) {
          dense_pc_ = param_value;
        } else {
          result.successful = false;
          result.reason = "Parameter is equal to the previous value. Do nothing.";
        }
      }
    }
  }
  return result;
}

void RealSenseD435::publishAlignedDepthTopic(const rs2::frame & frame, const rclcpp::Time & time)
{
  auto v_frame = frame.as<rs2::video_frame>();
  auto type = v_frame.get_profile().stream_type();
  auto width = v_frame.get_width();
  auto height = v_frame.get_height();

  cv::Mat cv_image = cv::Mat(height, width, CV_FORMAT.at(type));
  cv_image.data = const_cast<uchar *>(reinterpret_cast<const uchar *>(v_frame.get_data()));

  if (!node_.get_node_options().use_intra_process_comms()) {
    sensor_msgs::msg::Image::SharedPtr img;
    img = cv_bridge::CvImage(std_msgs::msg::Header(), MSG_ENCODING.at(type), cv_image).toImageMsg();
    //debug
    //RCLCPP_INFO(node_->get_logger(), "timestamp: %f, address: %p", t.seconds(), reinterpret_cast<std::uintptr_t>(img.get()));
    //
    img->header.frame_id = DEFAULT_ALIGNED_DEPTH_TO_COLOR_FRAME_ID;  //diff with publishTopics
    img->header.stamp = time;
    aligned_depth_image_pub_->publish(*img);  //diff
  } else {
    sensor_msgs::msg::Image::UniquePtr img;
    img = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, cv_image).toImageMsg(*img);
    //debug
    //RCLCPP_INFO(node_->get_logger(), "timestamp: %f, address: %p", t.seconds(), reinterpret_cast<std::uintptr_t>(img.get()));
    //
    img->header.frame_id = DEFAULT_ALIGNED_DEPTH_TO_COLOR_FRAME_ID;
    img->header.stamp = time;
    aligned_depth_image_pub_->publish(std::move(img));
  }
  
  //TODO: need to update calibration data if anything is changed dynamically.
  camera_info_[COLOR].header.stamp = time; //TODO: need to change frame id to the one for aligned depth frame(not color frame).
  aligned_depth_info_pub_->publish(camera_info_[COLOR]);
}

void RealSenseD435::publishSparsePointCloud(const rs2::points & points, const rs2::video_frame & color_frame, const rclcpp::Time & time)
{
  const rs2::vertex * vertex = points.get_vertices();
  const rs2::texture_coordinate * color_point = points.get_texture_coordinates();
  std::vector<int> valid_indices;

  for (size_t index = 0; index < points.size(); index++) {
    float i = color_point[index].u;
    float j = color_point[index].v;
    if (vertex[index].z>0 && i>=0 && i<=1 && j>=0 && j<=1) {
      valid_indices.push_back(index);
    }
  }
  //TODO: how to reduce some duplicated code?
  if (!node_.get_node_options().use_intra_process_comms()) {
    sensor_msgs::msg::PointCloud2::SharedPtr pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    //debug
    //RCLCPP_INFO(node_->get_logger(), "timestamp: %f, address: %p", t.seconds(), reinterpret_cast<std::uintptr_t>(pc_msg.get()));
    //
    pc_msg->header.stamp = time;
    pc_msg->header.frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
    pc_msg->width = valid_indices.size();
    pc_msg->height = 1;
    pc_msg->point_step = 3 * sizeof(float) + 3 * sizeof(uint8_t);
    pc_msg->row_step = pc_msg->point_step * pc_msg->width;
    pc_msg->is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(pc_msg->width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc_msg, "b");

    int width = color_frame.get_width();
    int height = color_frame.get_height();
    int channel_num = color_frame.get_bytes_per_pixel();
    uint8_t * color_data = (uint8_t*)color_frame.get_data();

    for (size_t index = 0; index < valid_indices.size(); index++) {
      auto pnt_idx = valid_indices[index];
      *iter_x = vertex[pnt_idx].x;
      *iter_y = vertex[pnt_idx].y;
      *iter_z = vertex[pnt_idx].z;

      float i = color_point[pnt_idx].u;
      float j = color_point[pnt_idx].v;
      int pixx = static_cast<int>(i * width);
      int pixy = static_cast<int>(j * height);
      int offset = (pixy * width + pixx) * channel_num;
      *iter_r = color_data[offset];
      *iter_g = color_data[offset+1];
      *iter_b = color_data[offset+2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
    pointcloud_pub_->publish(*pc_msg);
  } else {
    sensor_msgs::msg::PointCloud2::UniquePtr pc_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    //debug
    //RCLCPP_INFO(node_->get_logger(), "timestamp: %f, address: %p", t.seconds(), reinterpret_cast<std::uintptr_t>(pc_msg.get()));
    //
    pc_msg->header.stamp = time;
    pc_msg->header.frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
    pc_msg->width = valid_indices.size();
    pc_msg->height = 1;
    pc_msg->point_step = 3 * sizeof(float) + 3 * sizeof(uint8_t);
    pc_msg->row_step = pc_msg->point_step * pc_msg->width;
    pc_msg->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(pc_msg->width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc_msg, "b");

    int width = color_frame.get_width();
    int height = color_frame.get_height();
    int channel_num = color_frame.get_bytes_per_pixel();
    uint8_t * color_data = (uint8_t*)color_frame.get_data();

    for (size_t index = 0; index < valid_indices.size(); index++) {
      auto pnt_idx = valid_indices[index];
      *iter_x = vertex[pnt_idx].x;
      *iter_y = vertex[pnt_idx].y;
      *iter_z = vertex[pnt_idx].z;
      float i = color_point[pnt_idx].u;
      float j = color_point[pnt_idx].v;
      int pixx = static_cast<int>(i * width);
      int pixy = static_cast<int>(j * height);
      int offset = (pixy * width + pixx) * channel_num;
      *iter_r = color_data[offset];
      *iter_g = color_data[offset+1];
      *iter_b = color_data[offset+2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
    pointcloud_pub_->publish(std::move(pc_msg));
  }
}

void RealSenseD435::publishDensePointCloud(const rs2::points & points, const rs2::video_frame & color_frame, const rclcpp::Time & time)
{
  const rs2::vertex * vertex = points.get_vertices();

  if (!node_.get_node_options().use_intra_process_comms()) {
    sensor_msgs::msg::PointCloud2::SharedPtr pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    //debug
    //RCLCPP_INFO(node_->get_logger(), "timestamp: %f, address: %p", t.seconds(), reinterpret_cast<std::uintptr_t>(pc_msg.get()));
    //
    pc_msg->header.stamp = time;
    pc_msg->header.frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
    pc_msg->width = color_frame.get_width();
    pc_msg->height = color_frame.get_height();
    pc_msg->point_step = 3 * sizeof(float) + 3 * sizeof(uint8_t);
    pc_msg->row_step = pc_msg->point_step * pc_msg->width;
    pc_msg->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc_msg, "b");

    int channel_num = color_frame.get_bytes_per_pixel();
    uint8_t * color_data = (uint8_t*)color_frame.get_data();

    for (size_t pnt_idx = 0; pnt_idx < pc_msg->width*pc_msg->height; pnt_idx++) {
        *iter_x = vertex[pnt_idx].x;
        *iter_y = vertex[pnt_idx].y;
        *iter_z = vertex[pnt_idx].z;

        *iter_r = color_data[pnt_idx*channel_num];
        *iter_g = color_data[pnt_idx*channel_num+1];
        *iter_b = color_data[pnt_idx*channel_num+2];
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
      }
      pointcloud_pub_->publish(*pc_msg);
  } else {
    sensor_msgs::msg::PointCloud2::UniquePtr pc_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    //debug
    //RCLCPP_INFO(node_->get_logger(), "timestamp: %f, address: %p", t.seconds(), reinterpret_cast<std::uintptr_t>(pc_msg.get()));
    //
    pc_msg->header.stamp = time;
    pc_msg->header.frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
    pc_msg->width = color_frame.get_width();
    pc_msg->height =  color_frame.get_height();
    pc_msg->point_step = 3 * sizeof(float) + 3 * sizeof(uint8_t);
    pc_msg->row_step = pc_msg->point_step * pc_msg->width;
    pc_msg->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc_msg, "b");

    int channel_num = color_frame.get_bytes_per_pixel();
    uint8_t * color_data = (uint8_t*)color_frame.get_data();

    for (size_t pnt_idx = 0; pnt_idx < pc_msg->width*pc_msg->height; pnt_idx++) {
      *iter_x = vertex[pnt_idx].x;
      *iter_y = vertex[pnt_idx].y;
      *iter_z = vertex[pnt_idx].z;

      *iter_r = color_data[pnt_idx*channel_num];
      *iter_g = color_data[pnt_idx*channel_num+1];
      *iter_b = color_data[pnt_idx*channel_num+2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
    pointcloud_pub_->publish(std::move(pc_msg));
  }
}
}  // namespace realsense
