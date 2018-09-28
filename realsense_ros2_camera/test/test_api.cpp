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

// cpplint: c system headers
#include <gtest/gtest.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <realsense_ros2_camera/constants.hpp>
// cpplint: c++ system headers
#include <chrono>
#include <map>
#include <string>

bool g_enable_color = true;
bool g_color_recv = false;
float g_color_avg = 0.0f;

bool g_enable_depth = true;
bool g_depth_recv = false;

bool g_enable_infrared1 = true;
bool g_infrared1_recv = false;
float g_infrared1_avg = 0.0f;

bool g_enable_infrared2 = true;
bool g_infrared2_recv = false;
float g_infrared2_avg = 0.0f;

bool g_enable_pc = true;
bool g_pc_recv = false;
float g_pc_depth_avg = 0.0f;

bool g_tf_recv = false;
bool g_tf_color = false;

int g_fps = 0;
float g_latency = 0.0f;

int encoding2Mat(const std::string & encoding)
{
  std::map<std::string, int> map_encoding =
  {
    {"mono8", CV_8UC1},
    {"8UC1", CV_8UC1},
    {"bgr8", CV_8UC3},
    {"mono16", CV_16SC1},
    {"rgba8", CV_8UC4},
    {"bgra8", CV_8UC4},
    {"32FC1", CV_32FC1},
    {"rgb8", CV_8UC3},
    {"16UC1", CV_16UC1},
  };
  if (map_encoding.find(encoding) != map_encoding.end()) {
    return map_encoding[encoding];
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

void imageDepthCallback(const sensor_msgs::msg::Image::SharedPtr)
{
  g_depth_recv = true;
}

void imageColorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  static unsigned int fps = 0;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  static rclcpp::Time start = ros_clock.now();
  rclcpp::Time t(msg->header.stamp);
  g_latency = (ros_clock.now() - t).nanoseconds() / 1000000000.0f;
  cv::Mat frame(msg->height, msg->width, encoding2Mat(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  double color_total = 0.0;
  int channel = 0;
  cv::Scalar mean = cv::mean(frame);
  for (auto val : mean.val) {
    color_total += val;
    channel += 1;
  }

  if (channel > 0) {
    g_color_avg = color_total / channel;
  }

  g_color_recv = true;

  ++fps;
  if ((ros_clock.now() - start).nanoseconds() / 1000000.0f >= 1000) {
    g_fps = fps;
    fps = 0;
    start = ros_clock.now();
  }
}

void imageInfrared1Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat frame(msg->height, msg->width, encoding2Mat(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  double infrared1_total = 0.0;
  int channel = 0;
  cv::Scalar mean = cv::mean(frame);
  for (auto val : mean.val) {
    infrared1_total += val;
    channel += 1;
  }

  if (channel > 0) {
    g_infrared1_avg = infrared1_total / channel;
  }

  g_infrared1_recv = true;
}

void imageInfrared2Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat frame(msg->height, msg->width, encoding2Mat(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  uchar * infrared2_data = frame.data;
  double infrared2_total = 0.0;
  int infrared2_count = 1;

  for (unsigned int i = 0; i < msg->height * msg->width; i++) {
    if (*infrared2_data > 0 && *infrared2_data < 255) {
      infrared2_total += *infrared2_data;
      infrared2_count++;
    }
    infrared2_data++;
  }
  if (infrared2_count != 1) {
    g_infrared2_avg = static_cast<float>(infrared2_total / infrared2_count);
  }

  g_infrared2_recv = true;
}

void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  double pc_depth_total = 0.0;
  int pc_depth_count = 0;
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  for (unsigned int i = 0; i < msg->width * msg->height; ++i) {
    double pc_depth = std::ceil(*iter_z);
    if ((0.0 < pc_depth) && (pc_depth <= 10.0f * 100.0f)) {
      pc_depth_total += pc_depth;
      pc_depth_count++;
    }
    ++iter_z;
  }
  if (pc_depth_count != 0) {
    g_pc_depth_avg = static_cast<float>(pc_depth_total / pc_depth_count);
  }

  g_pc_recv = true;
}

void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (auto tf : msg->transforms) {
    if (tf.header.frame_id == "camera_color_frame") {
      g_tf_color = true;
    }
  }
  g_tf_recv = true;
}

TEST(TestAPI, testDepthStream) {
  if (g_enable_depth) {
    EXPECT_TRUE(g_depth_recv);
  } else {
    EXPECT_FALSE(g_depth_recv);
  }
}

TEST(TestAPI, testColorStream) {
  if (g_enable_color) {
    EXPECT_GT(g_color_avg, 0);
    EXPECT_TRUE(g_color_recv);
  } else {
    EXPECT_FALSE(g_color_recv);
  }
}

TEST(TestAPI, testInfrared1Stream) {
  if (g_enable_infrared1) {
    EXPECT_GT(g_infrared1_avg, 0);
    EXPECT_TRUE(g_infrared1_recv);
  } else {
    EXPECT_FALSE(g_infrared1_recv);
  }
}

TEST(TestAPI, testInfrared2Stream) {
  if (g_enable_infrared2) {
    EXPECT_GT(g_infrared2_avg, 0);
    EXPECT_TRUE(g_infrared2_recv);
  } else {
    EXPECT_FALSE(g_infrared2_recv);
  }
}

TEST(TestAPI, testPointCloud) {
  if (g_enable_pc) {
    EXPECT_GT(g_pc_depth_avg, 0);
    EXPECT_TRUE(g_pc_recv);
  } else {
    EXPECT_FALSE(g_pc_recv);
  }
}

TEST(TestAPI, testTF) {
  EXPECT_TRUE(g_tf_recv);
  EXPECT_TRUE(g_tf_color);
}

TEST(TestAPI, testLatency) {
  EXPECT_TRUE(g_latency > 0.0f);
}

TEST(TestAPI, testFPS) {
  EXPECT_GT(g_fps, 0);
}

int main(int argc, char * argv[]) try
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("realsense_test");
  auto sub1 = node->create_subscription<sensor_msgs::msg::Image>("camera/depth/image_rect_raw",
      imageDepthCallback, rmw_qos_profile_default);

  auto sub2 = node->create_subscription<sensor_msgs::msg::Image>("camera/color/image_raw",
      imageColorCallback, rmw_qos_profile_default);

  auto sub3 = node->create_subscription<sensor_msgs::msg::Image>("camera/infra1/image_rect_raw",
      imageInfrared1Callback, rmw_qos_profile_default);

  auto sub4 = node->create_subscription<sensor_msgs::msg::Image>("camera/infra2/image_rect_raw",
      imageInfrared2Callback, rmw_qos_profile_default);

  auto sub5 = node->create_subscription<sensor_msgs::msg::PointCloud2>("camera/depth/color/points",
      pcCallback, rmw_qos_profile_default);

  auto sub6 = node->create_subscription<tf2_msgs::msg::TFMessage>("tf_static",
      tfCallback, rmw_qos_profile_default);

  system("realsense_ros2_camera &");

  rclcpp::WallRate loop_rate(50);
  for (int i = 0; i < 300; ++i) {
    if (!rclcpp::ok()) {
      break;  // Break for ctrl-c
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  system("killall realsense_ros2_camera &");
  return RUN_ALL_TESTS();
} catch (...) {
}
