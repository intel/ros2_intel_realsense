// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>
#include "camera_core.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "opencv2/opencv.hpp"

void getCameraType()
{

  rs2::context ctx_;
  auto device_list = ctx_.query_devices();
  rs2::device_list & list = device_list;
  if (0 == list.size()) {
	std::cout <<  "No RealSense devices were found!" << std::endl;
  }
  for (auto && dev : list) {
    std::string pid_str = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
    uint16_t pid = std::stoi(pid_str, 0, 16);

    std::cout << "pid is" << pid << std::endl;

    switch(pid) {
      case RS435_RGB_PID:
        camera_type = "D435";
        break;
      case RS435i_RGB_PID:
        camera_type = "D435i";
        break;
      case RS_T265_PID:
        camera_type = "T265";
        break;
      default:
        break;
    }

    if (camera_type != "") 
      break;
  }

  std::cout << "camera_type is:" << camera_type << std::endl;
}

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

void getMsgInfo(rs2_stream stream, const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  g_encoding_recv[stream] = msg->encoding;
  g_width_recv[stream] = msg->width;
  g_height_recv[stream] = msg->height;
  g_step_recv[stream] = msg->step;
}

void getCameraInfo(rs2_stream stream, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  g_caminfo_height_recv[stream] = info_msg->height;
  g_caminfo_width_recv[stream] = info_msg->width;

  g_dmodel_recv[stream] = info_msg->distortion_model;

  // copy rotation matrix
  for (unsigned int i = 0; i < sizeof(info_msg->r) / sizeof(double); i++)
  {
    g_caminfo_rotation_recv[stream][i] = info_msg->r[i];
  }

  // copy projection matrix
  for (unsigned int i = 0; i < sizeof(info_msg->p) / sizeof(double); i++)
  {
    g_caminfo_projection_recv[stream][i] = info_msg->p[i];
  }
}

void imageDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv::Mat frame(msg->height, msg->width, encoding2Mat(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  double depth_total = 0;
  int depth_count = 0;
  cv::Scalar mean = cv::mean(frame);
  for (auto val : mean.val) {
    depth_total += static_cast<float>(val);
    depth_count += 1;
  }
  if (depth_count != 0)
  {
    g_depth_avg = static_cast<float>(depth_total / depth_count);
  }

  getMsgInfo(RS2_STREAM_DEPTH, msg);
  getCameraInfo(RS2_STREAM_DEPTH, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_depth_caminfo_D_recv[i] = info_msg->d[i];
  }

  g_depth_recv = true;
}

void imageColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
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

  getMsgInfo(RS2_STREAM_COLOR, msg);
  getCameraInfo(RS2_STREAM_COLOR, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_color_caminfo_D_recv[i] = info_msg->d[i];
  }

  g_color_recv = true;
}

void imageInfrared1Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv::Mat frame(msg->height, msg->width, encoding2Mat(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  double infrared1_total = 0.0;
  int infrared1_count = 0;
  cv::Scalar mean = cv::mean(frame);
  for (auto val : mean.val) {
    infrared1_total += val;
    infrared1_count += 1;
  }

  if (infrared1_count > 0) {
    g_infrared1_avg = infrared1_total / infrared1_count;
  }

  getMsgInfo(RS2_STREAM_INFRARED, msg);
  getCameraInfo(RS2_STREAM_INFRARED, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_infrared1_caminfo_D_recv[1] = info_msg->d[i];
  }

  g_infrared1_recv = true;
}

void imageInfrared2Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
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

  for (unsigned int i = 0; i < 5; i++)
  {
    g_infrared2_caminfo_D_recv[1] = info_msg->d[i];
  }
  g_infrared2_recv = true;
}

void imageFisheye1Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv::Mat frame(msg->height, msg->width, encoding2Mat(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  uchar *fisheye1_data = frame.data;
  double fisheye1_total = 0.0;
  int fisheye1_count = 1;
  for (unsigned int i = 0; i < msg->height * msg->width; i++)
  {
    if (*fisheye1_data > 0 && *fisheye1_data < 255)
    {
      fisheye1_total += *fisheye1_data;
      fisheye1_count++;
    }
    fisheye1_data++;
  }
  if (fisheye1_count != 0)
  {
    g_fisheye1_avg = static_cast<float>(fisheye1_total / fisheye1_count);
  }

  getMsgInfo(RS2_STREAM_FISHEYE, msg);
  getCameraInfo(RS2_STREAM_FISHEYE, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_fisheye1_caminfo_D_recv[i] = info_msg->d[i];
  }

  g_fisheye1_recv = true;
}

void imageFisheye2Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv::Mat frame(msg->height, msg->width, encoding2Mat(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  uchar *fisheye2_data = frame.data;
  double fisheye2_total = 0.0;
  int fisheye2_count = 1;
  for (unsigned int i = 0; i < msg->height * msg->width; i++)
  {
    if (*fisheye2_data > 0 && *fisheye2_data < 255)
    {
      fisheye2_total += *fisheye2_data;
      fisheye2_count++;
    }
    fisheye2_data++;
  }
  if (fisheye2_count != 0)
  {
    g_fisheye2_avg = static_cast<float>(fisheye2_total / fisheye2_count);
  }

  getMsgInfo(RS2_STREAM_FISHEYE, msg);
  getCameraInfo(RS2_STREAM_FISHEYE, info_msg);

  for (unsigned int i = 0; i < 5; i++)
  {
    g_fisheye2_caminfo_D_recv[i] = info_msg->d[i];
  }

  g_fisheye2_recv = true;
}
void imageAlignDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv::Mat frame(msg->height, msg->width, encoding2Mat(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  double alignDepth_total = 0.0;
  int alignDepth_count = 0;
  cv::Scalar mean = cv::mean(frame);
  for (auto val : mean.val) {
    alignDepth_total += static_cast<float>(val);
    alignDepth_count += 1;
  }
  if (alignDepth_count != 0)
  {
    g_alignDepth_avg = static_cast<float>(alignDepth_total / alignDepth_count);
  }

  for (unsigned int i = 0; i < 5; i++)
  {
    g_alignDepth_caminfo_D_recv[1] = info_msg->d[i];
  }

  g_alignDepth_recv = true;
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

void accelCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
   g_accel_recv = false;
   if (imu_msg->linear_acceleration_covariance[0] != -1.0)
   {
     if ((imu_msg->linear_acceleration.x != 0.000) || 
         (imu_msg->linear_acceleration.y != 0.000) ||
	 (imu_msg->linear_acceleration.y != 0.000))
     {
       g_accel_recv = true;
     }
   }
}

void gyroCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  g_gyro_recv = false;
  if (imu_msg->angular_velocity_covariance[0] != -1.0)
  {
    if ((imu_msg->angular_velocity.x != 0.0) ||
        (imu_msg->angular_velocity.y != 0.0) ||
        (imu_msg->angular_velocity.z != 0.0))
    {
      g_gyro_recv = true;
    }
  }
}

TEST(RealsenseTests, testIsColorStreamEnabled)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_color)
  {
    EXPECT_TRUE(g_color_recv);
  }
  else
  {
    EXPECT_FALSE(g_color_recv);
  }
}

TEST(RealsenseTests, testColorStream)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_color)
  {
    EXPECT_GT(g_color_avg, 0);
    if (!g_color_encoding_exp.empty())
    {
      EXPECT_EQ(g_color_encoding_exp, g_encoding_recv[RS2_STREAM_COLOR]);
    }
    if (g_color_step_exp > 0)
    {
      EXPECT_EQ(g_color_step_exp, g_step_recv[RS2_STREAM_COLOR]);
    }
  }
}

TEST(RealsenseTests, testColorResolution)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_color)
  {
    EXPECT_TRUE(g_color_recv);
    if (g_color_height_exp > 0)
    {
      EXPECT_EQ(g_color_height_exp, g_height_recv[RS2_STREAM_COLOR]);
    }
    if (g_color_width_exp > 0)
    {
      EXPECT_EQ(g_color_width_exp, g_width_recv[RS2_STREAM_COLOR]);
    }
  }
}

TEST(RealsenseTests, testColorCameraInfo)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_color)
  {
    EXPECT_EQ(g_width_recv[RS2_STREAM_COLOR], g_caminfo_width_recv[RS2_STREAM_COLOR]);
    EXPECT_EQ(g_height_recv[RS2_STREAM_COLOR], g_caminfo_height_recv[RS2_STREAM_COLOR]);
    EXPECT_STREQ(g_dmodel_recv[RS2_STREAM_COLOR].c_str(), "plumb_bob");

    // verify rotation is equal to identity matrix
    for (unsigned int i = 0; i < sizeof(ROTATION_IDENTITY) / sizeof(double); i++)
    {
      EXPECT_EQ(ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS2_STREAM_COLOR][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_COLOR][0] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_COLOR][1], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_COLOR][2] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_COLOR][3], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_COLOR][4], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_COLOR][5] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_COLOR][6] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_COLOR][7], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_COLOR][8], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_COLOR][9], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_COLOR][10] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_COLOR][11], 0.0);
  }
}

TEST(RealsenseTests, testIsDepthStreamEnabled)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_depth)
  {
    EXPECT_TRUE(g_depth_recv);
  }
  else
  {
    EXPECT_FALSE(g_depth_recv);
  }
}

TEST(RealsenseTests, testDepthStream)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_depth)
  {
    EXPECT_GT(g_depth_avg, 0);
    if (!g_depth_encoding_exp.empty())
    {
      EXPECT_EQ(g_depth_encoding_exp, g_encoding_recv[RS2_STREAM_DEPTH]);
    }
    if (g_depth_step_exp > 0)
    {
      EXPECT_EQ(g_depth_step_exp, g_step_recv[RS2_STREAM_DEPTH]);
    }
  }
}

TEST(RealsenseTests, testDepthResolution)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_depth)
  {
    EXPECT_TRUE(g_depth_recv);
    if (g_depth_height_exp > 0)
    {
      EXPECT_EQ(g_depth_height_exp, g_height_recv[RS2_STREAM_DEPTH]);
    }
    if (g_depth_width_exp > 0)
    {
      EXPECT_EQ(g_depth_width_exp, g_width_recv[RS2_STREAM_DEPTH]);
    }
  }
}

TEST(RealsenseTests, testDepthCameraInfo)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_depth)
  {
    EXPECT_EQ(g_width_recv[RS2_STREAM_DEPTH], g_caminfo_width_recv[RS2_STREAM_DEPTH]);
    EXPECT_EQ(g_height_recv[RS2_STREAM_DEPTH], g_caminfo_height_recv[RS2_STREAM_DEPTH]);
    EXPECT_STREQ(g_dmodel_recv[RS2_STREAM_DEPTH].c_str(), "plumb_bob");

    // verify rotation is equal to identity matrix
     for (unsigned int i = 0; i < sizeof(ROTATION_IDENTITY)/sizeof(double); i++)
     {
       EXPECT_EQ(ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS2_STREAM_DEPTH][i]);
     }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_DEPTH][0] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_DEPTH][1], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_DEPTH][2] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_DEPTH][3], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_DEPTH][4], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_DEPTH][5] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_DEPTH][6] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_DEPTH][7], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_DEPTH][8], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_DEPTH][9], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_DEPTH][10] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_DEPTH][11], 0.0);
  }
}

TEST(RealsenseTests, testIsInfrared1StreamEnabled)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_infrared1)
  {
    EXPECT_TRUE(g_infrared1_recv);
  }
  else
  {
    EXPECT_FALSE(g_infrared1_recv);
  }
}

TEST(RealsenseTests, testInfrared1Stream)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_infrared1)
  {
    EXPECT_GT(g_infrared1_avg, 0);
  }
}

TEST(RealsenseTests, testInfrared1Resolution)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_infrared1)
  {
    EXPECT_TRUE(g_infrared1_recv);
    if (g_infrared1_height_exp > 0)
    {
      EXPECT_EQ(g_infrared1_height_exp, g_height_recv[RS2_STREAM_INFRARED]);
    }
    if (g_infrared1_width_exp > 0)
    {
      EXPECT_EQ(g_infrared1_width_exp, g_width_recv[RS2_STREAM_INFRARED]);
    }
  }
}

TEST(RealsenseTests, testInfrared1CameraInfo)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_infrared1)
  {
    EXPECT_EQ(g_width_recv[RS2_STREAM_INFRARED], g_caminfo_width_recv[RS2_STREAM_INFRARED]);
    EXPECT_EQ(g_height_recv[RS2_STREAM_INFRARED], g_caminfo_height_recv[RS2_STREAM_INFRARED]);
    EXPECT_STREQ(g_dmodel_recv[RS2_STREAM_INFRARED].c_str(), "plumb_bob");

    // verify rotation is equal to identity matrix
    for (unsigned int i = 0; i < sizeof(ROTATION_IDENTITY) / sizeof(double); i++)
    {
      EXPECT_EQ(ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS2_STREAM_INFRARED][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_INFRARED][0] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_INFRARED][1], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_INFRARED][2] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_INFRARED][3], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_INFRARED][4], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_INFRARED][5] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_INFRARED][6] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_INFRARED][7], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_INFRARED][8], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_INFRARED][9], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_INFRARED][10] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_INFRARED][11], 0.0);
  }
}

TEST(RealsenseTests, testInfrared2Stream) 
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_infrared2) {
    EXPECT_GT(g_infrared2_avg, 0);
    EXPECT_TRUE(g_infrared2_recv);
  } else {
    EXPECT_FALSE(g_infrared2_recv);
  }
}

TEST(RealsenseTests, testIsFisheye1StreamEnabled)
{
  if (!camera_type.empty() && camera_type != "T265")
    GTEST_SKIP();
  if (g_enable_fisheye1)
  {
    EXPECT_TRUE(g_fisheye1_recv);
  }
  else
  {
    EXPECT_FALSE(g_fisheye1_recv);
  }
}

TEST(RealsenseTests, testFisheye1Stream)
{
  if (!camera_type.empty() && camera_type != "T265")
    GTEST_SKIP();
  if (g_enable_fisheye1)
  {
    EXPECT_GT(g_fisheye1_avg, 0);
  }
}

TEST(RealsenseTests, testFisheye1Resolution)
{
  if (!camera_type.empty() && camera_type != "T265")
    GTEST_SKIP();
  if (g_enable_fisheye1)
  {
    EXPECT_TRUE(g_fisheye1_recv);
    if (g_fisheye1_height_exp > 0)
    {
      EXPECT_EQ(g_fisheye1_height_exp, g_height_recv[RS2_STREAM_FISHEYE]);
    }
    if (g_fisheye1_width_exp > 0)
    {
      EXPECT_EQ(g_fisheye1_width_exp, g_width_recv[RS2_STREAM_FISHEYE]);
    }
  }
}

TEST(RealsenseTests, testFisheye1CameraInfo)
{
  if (!camera_type.empty() && camera_type != "T265")
    GTEST_SKIP();
  if (g_enable_fisheye1)
  {
    EXPECT_EQ(g_width_recv[RS2_STREAM_FISHEYE], g_caminfo_width_recv[RS2_STREAM_FISHEYE]);
    EXPECT_EQ(g_height_recv[RS2_STREAM_FISHEYE], g_caminfo_height_recv[RS2_STREAM_FISHEYE]);
    EXPECT_STREQ(g_dmodel_recv[RS2_STREAM_FISHEYE].c_str(), "plumb_bob");

    // verify rotation is equal to identity matrix
    for (unsigned int i = 0; i < sizeof(ROTATION_IDENTITY) / sizeof(double); i++)
    {
      EXPECT_EQ(ROTATION_IDENTITY[i], g_caminfo_rotation_recv[RS2_STREAM_FISHEYE][i]);
    }

    // check projection matrix values are set
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][0] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][1], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][2] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][3], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][4], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][5] != 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][6] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][7], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][8], 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][9], 0.0);
    EXPECT_TRUE(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][10] != 0.0);
    EXPECT_EQ(g_caminfo_projection_recv[RS2_STREAM_FISHEYE][11], 0.0); 

    // T265 cameras have Fisheye distortion parameters
    // Only the first coefficient is used/valid
    bool any_are_zero = false;
    for (unsigned int i = 0; i < 1; i++)
    {
      if (g_fisheye1_caminfo_D_recv[i] == 0.0)
      {
        any_are_zero = true;
      }
    }
    EXPECT_FALSE(any_are_zero);
  }
}

TEST(RealsenseTests, testFisheye2Stream) 
{
  if (!camera_type.empty() && camera_type != "T265")
    GTEST_SKIP();
  if (g_enable_fisheye2) {
    EXPECT_GT(g_fisheye2_avg, 0);
    EXPECT_TRUE(g_fisheye2_recv);
  } else {
    EXPECT_FALSE(g_fisheye2_recv);
  }
}

TEST(RealsenseTests, testPointCloud)
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_pc) {
    EXPECT_GT(g_pc_depth_avg, 0);
    EXPECT_TRUE(g_pc_recv);
  } else {
    EXPECT_FALSE(g_pc_recv);
  }
}

TEST(RealsenseTests, testAlignDepth) 
{
  if (!camera_type.empty() && camera_type != "D435" && camera_type != "D435i")
    GTEST_SKIP();
  if (g_enable_alignDepth) {
    EXPECT_GT(g_alignDepth_avg, 0);
    EXPECT_TRUE(g_alignDepth_recv);    
  } else {
    EXPECT_TRUE(g_alignDepth_recv);
  }
}

TEST(RealsenseTests, testImu)
{
  if (!camera_type.empty() && camera_type != "D435i" && camera_type != "T265")
    GTEST_SKIP();
  if (g_enable_imu)
  {
    EXPECT_TRUE(g_accel_recv);
    EXPECT_TRUE(g_gyro_recv);
  }
  else
  {
    EXPECT_FALSE(g_accel_recv);
    EXPECT_FALSE(g_gyro_recv);
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  getCameraType();
  auto node = rclcpp::Node::make_shared("realsense_camera");
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;

  RCLCPP_INFO(node->get_logger(), "Create a node for %s Camera", camera_type.c_str());
  auto depth_sub = image_transport::create_camera_subscription(node.get(), "/camera/depth/image_rect_raw", imageDepthCallback, "raw", custom_qos);
  auto color_sub = image_transport::create_camera_subscription(node.get(),"/camera/color/image_raw", imageColorCallback, "raw", custom_qos);
  auto infra1_sub = image_transport::create_camera_subscription(node.get(), "/camera/infra1/image_rect_raw", imageInfrared1Callback, "raw", custom_qos);
  auto infra2_sub = image_transport::create_camera_subscription(node.get(), "/camera/infra2/image_rect_raw", imageInfrared2Callback, "raw", custom_qos);
  auto alginDepth_sub = image_transport::create_camera_subscription(node.get(), "/camera/aligned_depth_to_color/image_raw", imageAlignDepthCallback, "raw", custom_qos);
  auto sub_pointcloud = node->create_subscription<sensor_msgs::msg::PointCloud2>("camera/pointcloud", rclcpp::QoS(1), pcCallback);
  auto sub_accel = node->create_subscription<sensor_msgs::msg::Imu>("/camera/accel/sample", rclcpp::QoS(1), accelCallback);
  auto sub_gyro = node->create_subscription<sensor_msgs::msg::Imu>("/camera/gyro/sample", rclcpp::QoS(1), gyroCallback);
  auto fisheye1_sub = image_transport::create_camera_subscription(node.get(), "/camera/fisheye1/image_raw", imageFisheye1Callback, "raw", custom_qos);
  auto fisheye2_sub = image_transport::create_camera_subscription(node.get(), "/camera/fisheye2/image_raw", imageFisheye2Callback, "raw", custom_qos);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const std::chrono::steady_clock::time_point max_runtime =
    std::chrono::steady_clock::now() + std::chrono::seconds(30);
  system("realsense_node &");
  while (rclcpp::ok())
  {
    if (std::chrono::steady_clock::now() >= max_runtime) {
      break;
    }
    exec.spin_once(std::chrono::seconds(1));
  }
  exec.remove_node(node);
  rclcpp::shutdown();
  system("killall realsense_node &");
  int ret = RUN_ALL_TESTS();
  return ret;
}
