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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/buffer_core.h"
#include "tf2/convert.h"
#include "realsense/rs_t265.hpp"

namespace realsense
{
RealSenseT265::RealSenseT265(rs2::context ctx, rs2::device dev, rclcpp::Node & node)
: RealSenseBase(ctx, dev, node)
{ 
  for (auto & stream : STREAMS) {
    setupStream(stream);
  }
 //  if (enable_[ACCEL] == true) {
  //   linear_accel_cov_ = node_.declare_parameter("accel0.linear_acceleration_covariance", DEFAULT_LINEAR_ACCEL_COV);
  // }
  // if (enable_[GYRO] == true) {
  //   angular_velocity_cov_ = node_.declare_parameter("gyro0.angular_velocity_covariance", DEFAULT_ANGULAR_VELOCITY_COV);
  // }
  linear_accel_cov_ = DEFAULT_LINEAR_ACCEL_COV;
  angular_velocity_cov_ = DEFAULT_ANGULAR_VELOCITY_COV;
  initialized_ = true;
}

void RealSenseT265::publishTopicsCallback(const rs2::frame & frame)
{
  rclcpp::Time t = node_.now();

  if (frame.is<rs2::frameset>()) {
    auto frameset = frame.as<rs2::frameset>();
    if (enable_[FISHEYE1] && (image_pub_[FISHEYE1]->get_subscription_count() > 0 || camera_info_pub_[FISHEYE1]->get_subscription_count() > 0)) {
      auto frame = frameset.get_fisheye_frame(1);
      publishImageTopic(frame, t);
    }
    if (enable_[FISHEYE2] && (image_pub_[FISHEYE2]->get_subscription_count() > 0 || camera_info_pub_[FISHEYE2]->get_subscription_count() > 0)) {
      auto frame = frameset.get_fisheye_frame(2);
      publishImageTopic(frame, t);
    }
  } else if (frame.is<rs2::motion_frame>()) {
    if ((enable_[ACCEL] && (imu_pub_[ACCEL]->get_subscription_count() > 0 || imu_info_pub_[ACCEL]->get_subscription_count() > 0))
      || (enable_[GYRO] && (imu_pub_[GYRO]->get_subscription_count() > 0 || imu_info_pub_[GYRO]->get_subscription_count() > 0))) {
      publishIMUTopic(frame, t);
    }
  } else if (frame.is<rs2::pose_frame>()) {
    if (enable_[POSE] && odom_pub_->get_subscription_count() > 0) {
      publishPoseTopic(frame, t);
    }
  }
}

Result RealSenseT265::paramChangeCallback(const std::vector<rclcpp::Parameter> & params)
{
  auto result = Result();
  result.successful = true;
  if (initialized_ == true) {
    for (auto & param : params) {
      auto param_name = param.get_name();
      if (param_name == "fisheye1.enabled") {
        result = toggleStream(FISHEYE1, param);
      } else if (param_name == "fisheye2.enabled") {
        result = toggleStream(FISHEYE2, param);
      } else if (param_name == "accel0.enabled") {
        result = toggleStream(ACCEL, param);
      } else if (param_name == "gyro0.enabled") {
        result = toggleStream(GYRO, param);
      } else if (param_name == "pose0.enabled") {
        result = toggleStream(POSE, param);
      }
    }
  }
  return result;
}

void RealSenseT265::publishIMUTopic(const rs2::frame & frame, const rclcpp::Time & time)
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

IMUInfo RealSenseT265::getIMUInfo(const rs2::frame & frame, const stream_index_pair & stream_index)
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

void RealSenseT265::publishPoseTopic(const rs2::frame & frame, const rclcpp::Time & time)
{
  auto type = frame.get_profile().stream_type();
  auto index = frame.get_profile().stream_index();
  auto type_index = std::pair<rs2_stream, int>(type, index);
  auto p_frame = frame.as<rs2::pose_frame>();
  rs2_pose pose = p_frame.get_pose_data();

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.pose.position.x = -pose.translation.z;
  pose_msg.pose.position.y = -pose.translation.x;
  pose_msg.pose.position.z = pose.translation.y;
  pose_msg.pose.orientation.x = -pose.rotation.z;
  pose_msg.pose.orientation.y = -pose.rotation.x;
  pose_msg.pose.orientation.z = pose.rotation.y;
  pose_msg.pose.orientation.w = pose.rotation.w;

  static tf2_ros::TransformBroadcaster br(node_);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = DEFAULT_ODOM_FRAME_ID;
  msg.child_frame_id = OPTICAL_FRAME_ID.at(type_index);
  msg.transform.translation.x = pose_msg.pose.position.x;
  msg.transform.translation.y = pose_msg.pose.position.y;
  msg.transform.translation.z = pose_msg.pose.position.z;
  msg.transform.rotation.x = pose_msg.pose.orientation.x;
  msg.transform.rotation.y = pose_msg.pose.orientation.y;
  msg.transform.rotation.z = pose_msg.pose.orientation.z;
  msg.transform.rotation.w = pose_msg.pose.orientation.w;

  br.sendTransform(msg);

  double cov_pose(linear_accel_cov_ * pow(10, 3-(int)pose.tracker_confidence));
  double cov_twist(angular_velocity_cov_ * pow(10, 1-(int)pose.tracker_confidence));

  geometry_msgs::msg::Vector3Stamped v_msg;
  tf2::Vector3 tfv(-pose.velocity.z,-pose.velocity.x, pose.velocity.y);
  tf2::Quaternion q(-msg.transform.rotation.x,-msg.transform.rotation.y,-msg.transform.rotation.z,msg.transform.rotation.w);
  tfv = tf2::quatRotate(q,tfv);
  v_msg.vector.x = tfv.x();
  v_msg.vector.y = tfv.y();
  v_msg.vector.z = tfv.z();

  geometry_msgs::msg::Vector3Stamped om_msg;
  tf2::Vector3 tfo(-pose.angular_velocity.z,-pose.angular_velocity.x, pose.angular_velocity.y);
  tfo = tf2::quatRotate(q,tfo);
  om_msg.vector.x = tfo.x();
  om_msg.vector.y = tfo.y();
  om_msg.vector.z = tfo.z();


  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.frame_id = DEFAULT_ODOM_FRAME_ID;
  odom_msg.child_frame_id = OPTICAL_FRAME_ID.at(type_index);
  odom_msg.header.stamp = time;
  odom_msg.pose.pose = pose_msg.pose;
  odom_msg.pose.covariance = {cov_pose, 0, 0, 0, 0, 0,
                              0, cov_pose, 0, 0, 0, 0,
                              0, 0, cov_pose, 0, 0, 0,
                              0, 0, 0, cov_twist, 0, 0,
                              0, 0, 0, 0, cov_twist, 0,
                              0, 0, 0, 0, 0, cov_twist};
  odom_msg.twist.twist.linear = v_msg.vector;
  odom_msg.twist.twist.angular = om_msg.vector;
  odom_msg.twist.covariance ={cov_pose, 0, 0, 0, 0, 0,
                              0, cov_pose, 0, 0, 0, 0,
                              0, 0, cov_pose, 0, 0, 0,
                              0, 0, 0, cov_twist, 0, 0,
                              0, 0, 0, 0, cov_twist, 0,
                              0, 0, 0, 0, 0, cov_twist};
  odom_pub_->publish(odom_msg);
}
}  // namespace realsense

