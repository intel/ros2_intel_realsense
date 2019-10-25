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

#ifndef REALSENSE__RS_BASE_HPP_
#define REALSENSE__RS_BASE_HPP_

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
#include "realsense/rs_constants.hpp"

namespace realsense
{
using Result = rcl_interfaces::msg::SetParametersResult;

class Float3
{
public:
  float x, y, z;

  Float3 & operator*=(const float & factor)
  {
    x *= factor;
    y *= factor;
    z *= factor;
    return (*this);
  }

  Float3 & operator+=(const Float3 & other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
    return (*this);
  }
};

class RealSenseBase
{
public:
  RealSenseBase(rs2::context ctx, rs2::device dev, rclcpp::Node & node);
  virtual ~RealSenseBase();
  virtual void publishTopicsCallback(const rs2::frame & frame) = 0;
  virtual Result paramChangeCallback(const std::vector<rclcpp::Parameter> & params) = 0;
  void startPipeline();
  void setupStream(const stream_index_pair & stream);
  void publishImageTopic(const rs2::frame & frame, const rclcpp::Time & time);
  void updateVideoStreamCalibData(const rs2::video_stream_profile & video_profile);
  void publishStaticTransforms(const rs2::stream_profile & base_profile, const std::vector<rs2::stream_profile> & active_profiles);
  void calculateTFAndPublish(const rs2::stream_profile & stream_in, const rs2::stream_profile & base_profile);
  void composeTFMsgAndPublish(const rclcpp::Time & t, const Float3 & translation,
                       const tf2::Quaternion & q, const std::string & from,
                       const std::string & to);
  tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;

  void printDeviceInfo();
  void printSupportedStreamProfiles();
  void printActiveStreamProfiles();
  void printStreamProfiles(const std::vector<rs2::stream_profile> & profile_list);

protected:
  Result toggleStream(const stream_index_pair & stream, const rclcpp::Parameter & param);
  Result changeResolution(const stream_index_pair & stream, const rclcpp::Parameter & param);
  Result changeFPS(const stream_index_pair & stream, const rclcpp::Parameter & param);

  typedef struct VideoStreamInfo
  {
    //rs2_format format;
    int width, height, fps;
    VideoStreamInfo(int w, int h, int f) {
      width = w;
      height = h;
      fps = f;
    }

    VideoStreamInfo() {}
  }VideoStreamInfo;

  rclcpp::Node & node_;
  rs2::context ctx_;
  rs2::device dev_;
  rs2::pipeline pipeline_;
  rs2::config cfg_;
  std::string base_frame_id_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<stream_index_pair, bool> enable_ = {{COLOR, false}, {DEPTH, false},
                                               {INFRA1, false}, {INFRA2, false},
                                               {ACCEL, false}, {GYRO, false},
                                               {FISHEYE1, false}, {FISHEYE2, false},
                                               {POSE, false}};

  std::map<stream_index_pair, VideoStreamInfo> stream_info_;
  std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> camera_info_;

  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pub_;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pub_;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_pub_;
  std::map<stream_index_pair, rclcpp::Publisher<realsense_msgs::msg::IMUInfo>::SharedPtr> imu_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
}  // namespace realsense
#endif  // REALSENSE__RS_BASE_HPP_
