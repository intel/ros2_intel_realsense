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

#include <sstream>
#include "realsense/rs_base.hpp"

namespace realsense
{
using namespace std::chrono_literals;

RealSenseBase::RealSenseBase(rs2::context ctx, rs2::device dev, rclcpp::Node & node)
: node_(node),
  ctx_(ctx),
  dev_(dev)
{
  // Publish static transforms
  if (node_.has_parameter("base_frame_id")) {
    node_.get_parameter("base_frame_id", base_frame_id_);
  } else {
    base_frame_id_ = node_.declare_parameter("base_frame_id", DEFAULT_BASE_FRAME_ID);
  }
  pipeline_ = rs2::pipeline(ctx_);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  node_.set_on_parameters_set_callback(std::bind(&RealSenseBase::paramChangeCallback, this, std::placeholders::_1));
}

RealSenseBase::~RealSenseBase()
{
  pipeline_.stop();
  if (work_thread_.joinable()) {
      work_thread_.join();
  }
}

void RealSenseBase::startWorkThread() 
{
  work_thread_ = std::thread([=]() {
    while (true) {
      rs2::frame frame = frame_data.wait_for_frame();
      publishTopicsCallback(frame);
    }
  });
}

void RealSenseBase::startPipeline()
{
  auto p_profile = cfg_.resolve(pipeline_);
  auto active_profiles = p_profile.get_streams();
  for (auto & profile : active_profiles) {
    if (profile.is<rs2::video_stream_profile>()) {
      updateVideoStreamCalibData(profile.as<rs2::video_stream_profile>());
    }
  }

  if (enable_[DEPTH] == true) {
    auto base_profile = p_profile.get_stream(RS2_STREAM_DEPTH, 0);
    auto pub_tf = [this, base_profile, active_profiles]() -> void {
      this->publishStaticTransforms(base_profile, active_profiles);
    };

    timer_ = node_.create_wall_timer(std::chrono::seconds(1), pub_tf);
  } else if (enable_[POSE] == true) {
    auto base_profile = p_profile.get_stream(RS2_STREAM_POSE, 0);
    publishStaticTransforms(base_profile, active_profiles);
  } else {
    RCLCPP_WARN(node_.get_logger(), "No TF is available. Enable base stream (Depth or Pose) first.");
  }

  frame_data = rs2::frame_queue(5);
  pipeline_.start(cfg_, frame_data);
  startWorkThread();
}

void RealSenseBase::setupStream(const stream_index_pair & stream)
{
  std::ostringstream os;
  os << STREAM_NAME.at(stream.first) << stream.second << ".enabled";
  bool enable;
  // TODO: find a better way to re-declare the parameters
  if (node_.has_parameter(os.str())) {
    node_.get_parameter(os.str(), enable);
  } else {
    enable = node_.declare_parameter(os.str(), DEFAULT_ENABLE_STREAM);
  }

  if (stream == ACCEL || stream == GYRO) {
    imu_pub_.insert(std::pair<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr>
      (stream, node_.create_publisher<sensor_msgs::msg::Imu>(SAMPLE_TOPIC.at(stream), rclcpp::QoS(1))));
    imu_info_pub_.insert(std::pair<stream_index_pair, rclcpp::Publisher<realsense_msgs::msg::IMUInfo>::SharedPtr>
      (stream, node_.create_publisher<realsense_msgs::msg::IMUInfo>(INFO_TOPIC.at(stream), rclcpp::QoS(1))));
    if (enable == true) {
      enable_[stream] = true;
      cfg_.enable_stream(stream.first, stream.second);
    }
  } else if (stream == POSE) {
    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(SAMPLE_TOPIC.at(stream), rclcpp::QoS(1));
    if (enable == true) {
      enable_[stream] = true;
      cfg_.enable_stream(stream.first, stream.second);
    }
  } else {
    std::vector<int64_t> res;
    int fps;
    if (stream == COLOR || stream == DEPTH || stream == INFRA1 || stream == INFRA2) {
      os.str("");
      os << STREAM_NAME.at(stream.first) << stream.second << ".resolution";
      if (node_.has_parameter(os.str())) {
        node_.get_parameter(os.str(), res);
      } else {
        res = node_.declare_parameter(os.str(), rclcpp::ParameterValue(DEFAULT_IMAGE_RESOLUTION)).get<rclcpp::PARAMETER_INTEGER_ARRAY>();      
      }
      os.str("");
      os << STREAM_NAME.at(stream.first) << stream.second << ".fps";
      if (node_.has_parameter(os.str())) {
        node_.get_parameter(os.str(), fps);} else {
        fps = node_.declare_parameter(os.str(), DEFAULT_IMAGE_FPS);    
      }
    } else if (stream == FISHEYE1 || stream == FISHEYE2) {
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
      param_desc.read_only = true;
      os.str("");
      os << STREAM_NAME.at(stream.first) << stream.second << ".resolution";
      if (node_.has_parameter(os.str())) {
        node_.get_parameter(os.str(), res);
      } else {
        res = node_.declare_parameter(os.str(), rclcpp::ParameterValue(FISHEYE_RESOLUTION), param_desc).get<rclcpp::PARAMETER_INTEGER_ARRAY>();
      }
      os.str("");
      os << STREAM_NAME.at(stream.first) << stream.second << ".fps";
      if (node_.has_parameter(os.str())) {
        node_.get_parameter(os.str(), fps);
      } else {        
        fps = node_.declare_parameter(os.str(), DEFAULT_IMAGE_FPS, param_desc);
      }
    }

    VideoStreamInfo info(static_cast<int>(res[0]), static_cast<int>(res[1]), fps);

    stream_info_.insert(std::pair<stream_index_pair, VideoStreamInfo>(stream, info));
    image_pub_.insert(std::pair<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
      (stream, node_.create_publisher<sensor_msgs::msg::Image>(SAMPLE_TOPIC.at(stream), rclcpp::QoS(1))));
    camera_info_pub_.insert(std::pair<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
      (stream, node_.create_publisher<sensor_msgs::msg::CameraInfo>(INFO_TOPIC.at(stream), rclcpp::QoS(1))));
    if (enable == true) {
      enable_[stream] = true;
      cfg_.enable_stream(stream.first, stream.second, info.width, info.height, STREAM_FORMAT.at(stream.first), info.fps);
    }
  }
}

void RealSenseBase::publishImageTopic(const rs2::frame & frame, const rclcpp::Time & time)
{
  auto v_frame = frame.as<rs2::video_frame>();
  auto type = v_frame.get_profile().stream_type();
  auto index = v_frame.get_profile().stream_index();
  auto type_index = std::pair<rs2_stream, int>(type, index);
  auto width = v_frame.get_width();
  auto height = v_frame.get_height();

  cv::Mat cv_image = cv::Mat(height, width, CV_FORMAT.at(type));
  cv_image.data = const_cast<uchar *>(reinterpret_cast<const uchar *>(v_frame.get_data()));

  if (!node_.get_node_options().use_intra_process_comms()) {
    sensor_msgs::msg::Image::SharedPtr img;
    img = cv_bridge::CvImage(std_msgs::msg::Header(), MSG_ENCODING.at(type), cv_image).toImageMsg();
    //debug
    //RCLCPP_INFO(node_.get_logger(), "non-intra: timestamp: %f, address: %p", time.seconds(), reinterpret_cast<std::uintptr_t>(img.get()));
    //
    img->header.frame_id = OPTICAL_FRAME_ID.at(type_index);
    img->header.stamp = time;
    image_pub_[type_index]->publish(*img);
  } else {
    auto img = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(std_msgs::msg::Header(), MSG_ENCODING.at(type), cv_image).toImageMsg(*img);
    //debug
    //RCLCPP_INFO(node_.get_logger(), "intra: timestamp: %f, address: %p", time.seconds(), reinterpret_cast<std::uintptr_t>(img.get()));
    //
    img->header.frame_id = OPTICAL_FRAME_ID.at(type_index);
    img->header.stamp = time;
    image_pub_[type_index]->publish(std::move(img));
  }
  //TODO: need to update calibration data if anything is changed dynamically.
  camera_info_[type_index].header.stamp = time;
  camera_info_pub_[type_index]->publish(camera_info_[type_index]);
}

void RealSenseBase::updateVideoStreamCalibData(const rs2::video_stream_profile & video_profile)
{
  stream_index_pair type_index{video_profile.stream_type(), video_profile.stream_index()};
  auto intrinsic = video_profile.get_intrinsics();
  camera_info_[type_index].width = intrinsic.width;
  camera_info_[type_index].height = intrinsic.height;
  camera_info_[type_index].header.frame_id = OPTICAL_FRAME_ID.at(type_index);

  camera_info_[type_index].k.at(0) = intrinsic.fx;
  camera_info_[type_index].k.at(2) = intrinsic.ppx;
  camera_info_[type_index].k.at(4) = intrinsic.fy;
  camera_info_[type_index].k.at(5) = intrinsic.ppy;
  camera_info_[type_index].k.at(8) = 1;

  camera_info_[type_index].p.at(0) = camera_info_[type_index].k.at(0);
  camera_info_[type_index].p.at(1) = 0;
  camera_info_[type_index].p.at(2) = camera_info_[type_index].k.at(2);
  camera_info_[type_index].p.at(3) = 0;
  camera_info_[type_index].p.at(4) = 0;
  camera_info_[type_index].p.at(5) = camera_info_[type_index].k.at(4);
  camera_info_[type_index].p.at(6) = camera_info_[type_index].k.at(5);
  camera_info_[type_index].p.at(7) = 0;
  camera_info_[type_index].p.at(8) = 0;
  camera_info_[type_index].p.at(9) = 0;
  camera_info_[type_index].p.at(10) = 1;
  camera_info_[type_index].p.at(11) = 0;

  camera_info_[type_index].distortion_model = "plumb_bob";

  // set R (rotation matrix) values to identity matrix
  camera_info_[type_index].r.at(0) = 1.0;
  camera_info_[type_index].r.at(1) = 0.0;
  camera_info_[type_index].r.at(2) = 0.0;
  camera_info_[type_index].r.at(3) = 0.0;
  camera_info_[type_index].r.at(4) = 1.0;
  camera_info_[type_index].r.at(5) = 0.0;
  camera_info_[type_index].r.at(6) = 0.0;
  camera_info_[type_index].r.at(7) = 0.0;
  camera_info_[type_index].r.at(8) = 1.0;

  camera_info_[type_index].d.resize(5);
  for (int i = 0; i < 5; i++) {
    camera_info_[type_index].d.at(i) = intrinsic.coeffs[i];
  }

  if (type_index == DEPTH && enable_[DEPTH] && enable_[COLOR]) {
      camera_info_[type_index].p.at(3) = 0;     // Tx
      camera_info_[type_index].p.at(7) = 0;     // Ty
  }
}

void RealSenseBase::publishStaticTransforms(const rs2::stream_profile & base_profile, const std::vector<rs2::stream_profile> & active_profiles)
{
  for (auto & profile : active_profiles) {
    calculateTFAndPublish(profile, base_profile);
  }
}

void RealSenseBase::calculateTFAndPublish(const rs2::stream_profile & stream_in, const rs2::stream_profile & base_profile)
{
  tf2::Quaternion quaternion_optical;
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  rclcpp::Time transform_ts = node_.now();

  rs2_extrinsics ex;
  try {
    ex = stream_in.get_extrinsics_to(base_profile);
  } catch (std::exception& e) {
    if (!strcmp(e.what(), "Requested extrinsics are not available!")) {
      RCLCPP_WARN(node_.get_logger(), "%s : using unity as default.", e.what());
      ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
    } else {
      throw e;
    }
  }
  auto Q = rotationMatrixToQuaternion(ex.rotation);
  Q = quaternion_optical * Q * quaternion_optical.inverse();

  Float3 translation{ex.translation[0], ex.translation[1], ex.translation[2]};
  auto type = stream_in.stream_type();
  auto index = stream_in.stream_index();
  auto type_index = std::pair<rs2_stream, int>(type, index);
  if (type == RS2_STREAM_POSE) {
    Q = Q.inverse();
    composeTFMsgAndPublish(transform_ts, translation, Q, OPTICAL_FRAME_ID.at(type_index), base_frame_id_);
  } else {
    composeTFMsgAndPublish(transform_ts, translation, quaternion_optical, base_frame_id_, OPTICAL_FRAME_ID.at(type_index));
  }
}

void RealSenseBase::composeTFMsgAndPublish(const rclcpp::Time & t, const Float3 & translation,
                                           const tf2::Quaternion & q, const std::string & from,
                                           const std::string & to)
{
  geometry_msgs::msg::TransformStamped msg;
  RCLCPP_DEBUG(node_.get_logger(), "Publish Static TF from %s to %s", from.c_str(), to.c_str());
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = translation.z;
  msg.transform.translation.y = -translation.x;
  msg.transform.translation.z = -translation.y;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();
  static_tf_broadcaster_-> sendTransform(msg);
}

tf2::Quaternion RealSenseBase::rotationMatrixToQuaternion(const float rotation[9]) const
{
  Eigen::Matrix3f m;
  // We need to be careful about the order, as RS2 rotation matrix is
  // column-major, while Eigen::Matrix3f expects row-major.
  m << rotation[0], rotation[3], rotation[6],
       rotation[1], rotation[4], rotation[7],
       rotation[2], rotation[5], rotation[8];
  Eigen::Quaternionf q(m);
  return tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
}

void RealSenseBase::printDeviceInfo()
{
  auto camera_name = dev_.get_info(RS2_CAMERA_INFO_NAME);
  RCLCPP_INFO(node_.get_logger(), "+++++++++++++++++++++");
  RCLCPP_INFO(node_.get_logger(), "Device Name: %s", camera_name);
  auto serial_no = dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  RCLCPP_INFO(node_.get_logger(), "Device Serial No: %s", serial_no);
  auto fw_ver = dev_.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
  RCLCPP_INFO(node_.get_logger(), "Device FW Version: %s", fw_ver);
  auto pid = dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
  RCLCPP_INFO(node_.get_logger(), "Device Product ID: 0x%s", pid);
  RCLCPP_INFO(node_.get_logger(), "+++++++++++++++++++++");
}

void RealSenseBase::printSupportedStreamProfiles()
{
  auto sensor_list = dev_.query_sensors();
  
  for(auto sensor : sensor_list) {
    RCLCPP_INFO(node_.get_logger(), "Sensor Name: %s", sensor.get_info(RS2_CAMERA_INFO_NAME));
    auto profile_list = sensor.get_stream_profiles();
    printStreamProfiles(profile_list);
  }
}

void RealSenseBase::printActiveStreamProfiles()
{
  auto p_profile = pipeline_.get_active_profile();
  auto profile_list = p_profile.get_streams();
  printStreamProfiles(profile_list);
}

void RealSenseBase::printStreamProfiles(const std::vector<rs2::stream_profile> & profile_list)
{
  for (auto profile : profile_list) {
      auto p = profile.as<rs2::video_stream_profile>();
      RCLCPP_INFO(node_.get_logger(), "+++++++++++++++++++++");
      RCLCPP_INFO(node_.get_logger(), "Stream Name: %s", p.stream_name().c_str());
      RCLCPP_INFO(node_.get_logger(), "Type: %s", rs2_stream_to_string(p.stream_type()));
      RCLCPP_INFO(node_.get_logger(), "Index: %d", p.stream_index());
      RCLCPP_INFO(node_.get_logger(), "Unique id: %d", p.unique_id());
      RCLCPP_INFO(node_.get_logger(), "Format: %s", rs2_format_to_string(p.format()));
      RCLCPP_INFO(node_.get_logger(), "Width: %d", p.width());
      RCLCPP_INFO(node_.get_logger(), "Height: %d", p.height());
      RCLCPP_INFO(node_.get_logger(), "FPS: %d", p.fps());
  }
}

Result RealSenseBase::toggleStream(const stream_index_pair & stream, const rclcpp::Parameter & param)
{
  auto result = Result();
  result.successful = true;
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    result.successful = false;
    result.reason = "Type should be boolean.";
    return result;
  }
  if (param.as_bool() == true && enable_[stream] == false) {
    if (stream == ACCEL || stream == GYRO || stream == POSE) {
      cfg_.enable_stream(stream.first, stream.second);
    } else {
      cfg_.enable_stream(stream.first, stream.second, stream_info_[stream].width, stream_info_[stream].height,
      STREAM_FORMAT.at(stream.first), stream_info_[stream].fps);
    }
    pipeline_.stop();
    rclcpp::sleep_for(200ms);
    pipeline_.start(cfg_, std::bind(&RealSenseBase::publishTopicsCallback, this, std::placeholders::_1));
    enable_[stream] = true;
    //Publish TF
    auto p_profile = cfg_.resolve(pipeline_);
    auto active_profiles = p_profile.get_streams();
    if (enable_[DEPTH] == true) {
      auto base_profile = p_profile.get_stream(RS2_STREAM_DEPTH, 0);
      publishStaticTransforms(base_profile, active_profiles);
    } else if (enable_[POSE] == true) {
      auto base_profile = p_profile.get_stream(RS2_STREAM_POSE, 0);
      publishStaticTransforms(base_profile, active_profiles);
    } else {
      RCLCPP_WARN(node_.get_logger(), "No TF is available. Enable base stream (Depth or Pose) first.");
    }
    RCLCPP_INFO(node_.get_logger(), "%s stream is enabled.", STREAM_NAME.at(stream.first).c_str());
  } else if (param.as_bool() == false && enable_[stream] == true) {
    cfg_.disable_stream(stream.first, stream.second);
    enable_[stream] = false;
    RCLCPP_INFO(node_.get_logger(), "%s stream is disabled.", STREAM_NAME.at(stream.first).c_str());
  } else {
    result.successful = false;
    result.reason = "Parameter is equal to the previous value. Do nothing.";
  }
  return result;
}

Result RealSenseBase::changeResolution(const stream_index_pair & stream, const rclcpp::Parameter & param)
{
  auto result = Result();
  result.successful = true;
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    result.successful = false;
    result.reason = "Type should be integer array.";
    return result;
  }
  auto res = param.as_integer_array();
  cfg_.enable_stream(stream.first, stream.second, res[0], res[1], STREAM_FORMAT.at(stream.first), stream_info_[stream].fps);
  if (cfg_.can_resolve(pipeline_)) {
    if (enable_[stream] == true) {
      pipeline_.stop();
      pipeline_.start(cfg_, std::bind(&RealSenseBase::publishTopicsCallback, this, std::placeholders::_1));
    }
    stream_info_[stream].width = static_cast<int>(res[0]);
    stream_info_[stream].height = static_cast<int>(res[1]);
  } else {
    result.successful = false;
    result.reason = "Unsupported resolution.";
  }
  return result;
}

Result RealSenseBase::changeFPS(const stream_index_pair & stream, const rclcpp::Parameter & param)
{
  auto result = Result();
  result.successful = true;
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
    result.successful = false;
    result.reason = "Type should be integer.";
    return result;
  }
  int fps = param.as_int();
  cfg_.enable_stream(stream.first, stream.second, stream_info_[stream].width, stream_info_[stream].height,
        STREAM_FORMAT.at(stream.first), fps);
  if (cfg_.can_resolve(pipeline_)) {
    if (enable_[stream] == true) {
      pipeline_.stop();
      pipeline_.start(cfg_, std::bind(&RealSenseBase::publishTopicsCallback, this, std::placeholders::_1));
    }
    stream_info_[stream].fps = fps;
  } else {
    result.successful = false;
    result.reason = "Unsupported configuration.";
  }
  return result;
}
}  // namespace realsense
