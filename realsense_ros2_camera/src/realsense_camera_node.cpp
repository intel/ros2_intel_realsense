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
#include <eigen3/Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <console_bridge/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
// cpplint: c++ system headers
#include <algorithm>
#include <csignal>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
// cpplint: other headers
#include "realsense_ros2_camera/constants.hpp"
#include "realsense_camera_msgs/msg/imu_info.hpp"
#include "realsense_camera_msgs/msg/extrinsics.hpp"


#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION. \
  REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros2_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;
using realsense_camera_msgs::msg::Extrinsics;
using realsense_camera_msgs::msg::IMUInfo;

namespace realsense_ros2_camera
{
using stream_index_pair = std::pair<rs2_stream, int>;

const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};

const std::vector<std::vector<stream_index_pair>> IMAGE_STREAMS = {{{DEPTH, INFRA1, INFRA2},
  {COLOR}, {FISHEYE}}};

const std::vector<std::vector<stream_index_pair>> HID_STREAMS = {{GYRO, ACCEL}};

rs2::device _dev;
inline void signalHandler(int signum)
{
  std::cout << strsignal(signum) << " Signal is received! Terminate RealSense Node...\n";
  auto sens = _dev.query_sensors();
  for (auto it = sens.begin(); it != sens.end(); it++) {
    it->stop();
    it->close();
  }
  rclcpp::shutdown();
  exit(signum);
}

class PipelineSyncer : public rs2::asynchronous_syncer
{
public:
  void operator()(rs2::frame f) const
  {
    invoke(std::move(f));
  }
};

class RealSenseCameraNode : public rclcpp::Node
{
public:
  RealSenseCameraNode()
  : Node("RealSenseCameraNode",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    _ros_clock(RCL_ROS_TIME),
    _serial_no(""),
    _base_frame_id(""),
    qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)),
    _intialize_time_base(false)
  {
    RCLCPP_INFO(logger_, "RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
    RCLCPP_INFO(logger_, "Running with LibRealSense v%s", RS2_API_VERSION_STR);

    signal(SIGINT, signalHandler);
    auto severity = rs2_log_severity::RS2_LOG_SEVERITY_ERROR;
    tryGetLogSeverity(severity);
    if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity) {
      console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
    }

    rs2::log_to_console(severity);

    // Types for depth stream
    _format[DEPTH] = RS2_FORMAT_Z16;           // libRS type
    _image_format[DEPTH] = CV_16UC1;            // CVBridge type
    _encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;         // ROS message type
    _unit_step_size[DEPTH] = sizeof(uint16_t);         // sensor_msgs::ImagePtr row step size
    _stream_name[DEPTH] = "depth";

    // Infrared stream - Left
    _format[INFRA1] = RS2_FORMAT_Y8;           // libRS type
    _image_format[INFRA1] = CV_8UC1;            // CVBridge type
    _encoding[INFRA1] = sensor_msgs::image_encodings::TYPE_8UC1;         // ROS message type
    _unit_step_size[INFRA1] = sizeof(uint8_t);         // sensor_msgs::ImagePtr row step size
    _stream_name[INFRA1] = "infra1";

    // Infrared stream - Right
    _format[INFRA2] = RS2_FORMAT_Y8;           // libRS type
    _image_format[INFRA2] = CV_8UC1;            // CVBridge type
    _encoding[INFRA2] = sensor_msgs::image_encodings::TYPE_8UC1;         // ROS message type
    _unit_step_size[INFRA2] = sizeof(uint8_t);         // sensor_msgs::ImagePtr row step size
    _stream_name[INFRA2] = "infra2";

    // Types for color stream
    _format[COLOR] = RS2_FORMAT_RGB8;           // libRS type
    _image_format[COLOR] = CV_8UC3;            // CVBridge type
    _encoding[COLOR] = sensor_msgs::image_encodings::RGB8;         // ROS message type
    _unit_step_size[COLOR] = 3;         // sensor_msgs::ImagePtr row step size
    _stream_name[COLOR] = "color";

    // Types for fisheye stream
    _format[FISHEYE] = RS2_FORMAT_RAW8;           // libRS type
    _image_format[FISHEYE] = CV_8UC1;            // CVBridge type
    _encoding[FISHEYE] = sensor_msgs::image_encodings::TYPE_8UC1;         // ROS message type
    _unit_step_size[FISHEYE] = sizeof(uint8_t);         // sensor_msgs::ImagePtr row step size
    _stream_name[FISHEYE] = "fisheye";

    // Types for Motion-Module streams
    _format[GYRO] = RS2_FORMAT_MOTION_XYZ32F;           // libRS type
    _image_format[GYRO] = CV_8UC1;            // CVBridge type
    _encoding[GYRO] = sensor_msgs::image_encodings::TYPE_8UC1;         // ROS message type
    _unit_step_size[GYRO] = sizeof(uint8_t);         // sensor_msgs::ImagePtr row step size
    _stream_name[GYRO] = "gyro";

    _format[ACCEL] = RS2_FORMAT_MOTION_XYZ32F;           // libRS type
    _image_format[ACCEL] = CV_8UC1;            // CVBridge type
    _encoding[ACCEL] = sensor_msgs::image_encodings::TYPE_8UC1;         // ROS message type
    _unit_step_size[ACCEL] = sizeof(uint8_t);         // sensor_msgs::ImagePtr row step size
    _stream_name[ACCEL] = "accel";
  }

  virtual ~RealSenseCameraNode()
  {}

  virtual void onInit()
  {
    getParameters();
    setupDevice();
    setupPublishers();
    setupStreams();
    rclcpp::sleep_for(std::chrono::nanoseconds(2000000000));
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RealSenseCameraNode::publishStaticTransforms, this));
    RCLCPP_INFO(logger_, "RealSense Node Is Up!");
  }

private:
  void getParameters()
  {
    RCLCPP_INFO(logger_, "getParameters...");

    this->get_parameter_or("enable_pointcloud", _pointcloud, POINTCLOUD);
    this->get_parameter_or("enable_aligned_pointcloud", _align_pointcloud, ALIGN_POINTCLOUD);
    // this->get_parameter_or("enable_sync", _sync_frames, SYNC_FRAMES);
    this->get_parameter_or("enable_depth", _enable[DEPTH], ENABLE_DEPTH);
    this->get_parameter_or("enable_aligned_depth", _align_depth, ALIGN_DEPTH);
    this->get_parameter_or("enable_infra1", _enable[INFRA1], ENABLE_INFRA1);
    this->get_parameter_or("enable_infra2", _enable[INFRA2], ENABLE_INFRA2);
    if (!_enable[DEPTH]) {
      _pointcloud = false;
      _align_depth = false;
      _enable[INFRA1] = false;
      _enable[INFRA2] = false;
    }

    if (!_enable[DEPTH] || !_align_depth) {
      _align_pointcloud = false;
    }

    if (_pointcloud || _align_depth) {
      _sync_frames = true;
    } else {
      _sync_frames = false;
    }
    this->get_parameter("serial_no", _serial_no);

    this->get_parameter_or("depth_width", _width[DEPTH], DEPTH_WIDTH);
    this->get_parameter_or("depth_height", _height[DEPTH], DEPTH_HEIGHT);
    this->get_parameter_or("depth_fps", _fps[DEPTH], DEPTH_FPS);

    this->get_parameter_or("infra1_width", _width[INFRA1], INFRA1_WIDTH);
    this->get_parameter_or("infra1_height", _height[INFRA1], INFRA1_HEIGHT);
    this->get_parameter_or("infra1_fps", _fps[INFRA1], INFRA1_FPS);

    this->get_parameter_or("infra2_width", _width[INFRA2], INFRA2_WIDTH);
    this->get_parameter_or("infra2_height", _height[INFRA2], INFRA2_HEIGHT);
    this->get_parameter_or("infra2_fps", _fps[INFRA2], INFRA2_FPS);

    this->get_parameter_or("color_width", _width[COLOR], COLOR_WIDTH);
    this->get_parameter_or("color_height", _height[COLOR], COLOR_HEIGHT);
    this->get_parameter_or("color_fps", _fps[COLOR], COLOR_FPS);
    this->get_parameter_or("enable_color", _enable[COLOR], ENABLE_COLOR);

    this->get_parameter_or("fisheye_width", _width[FISHEYE], FISHEYE_WIDTH);
    this->get_parameter_or("fisheye_height", _height[FISHEYE], FISHEYE_HEIGHT);
    this->get_parameter_or("fisheye_fps", _fps[FISHEYE], FISHEYE_FPS);
    this->get_parameter_or("enable_fisheye", _enable[FISHEYE], ENABLE_FISHEYE);

    this->get_parameter_or("gyro_fps", _fps[GYRO], GYRO_FPS);
    this->get_parameter_or("accel_fps", _fps[ACCEL], ACCEL_FPS);
    this->get_parameter_or("enable_imu", _enable[GYRO], ENABLE_IMU);
    this->get_parameter_or("enable_imu", _enable[ACCEL], ENABLE_IMU);

    this->get_parameter_or("base_frame_id", _base_frame_id,
      std::string(DEFAULT_BASE_FRAME_ID));
    this->get_parameter_or("depth_frame_id", _frame_id[DEPTH],
      std::string(DEFAULT_DEPTH_FRAME_ID));
    this->get_parameter_or("infra1_frame_id", _frame_id[INFRA1],
      std::string(DEFAULT_INFRA1_FRAME_ID));
    this->get_parameter_or("infra2_frame_id", _frame_id[INFRA2],
      std::string(DEFAULT_INFRA2_FRAME_ID));
    this->get_parameter_or("color_frame_id", _frame_id[COLOR],
      std::string(DEFAULT_COLOR_FRAME_ID));
    this->get_parameter_or("fisheye_frame_id", _frame_id[FISHEYE],
      std::string(DEFAULT_FISHEYE_FRAME_ID));
    this->get_parameter_or("imu_gyro_frame_id", _frame_id[GYRO],
      std::string(DEFAULT_IMU_FRAME_ID));
    this->get_parameter_or("imu_accel_frame_id", _frame_id[ACCEL],
      std::string(DEFAULT_IMU_FRAME_ID));

    this->get_parameter_or("depth_optical_frame_id", _optical_frame_id[DEPTH],
      std::string(DEFAULT_DEPTH_OPTICAL_FRAME_ID));
    this->get_parameter_or("infra1_optical_frame_id", _optical_frame_id[INFRA1],
      std::string(DEFAULT_INFRA1_OPTICAL_FRAME_ID));
    this->get_parameter_or("infra2_optical_frame_id", _optical_frame_id[INFRA2],
      std::string(DEFAULT_INFRA2_OPTICAL_FRAME_ID));
    this->get_parameter_or("color_optical_frame_id", _optical_frame_id[COLOR],
      std::string(DEFAULT_COLOR_OPTICAL_FRAME_ID));
    this->get_parameter_or("fisheye_optical_frame_id", _optical_frame_id[FISHEYE],
      std::string(DEFAULT_FISHEYE_OPTICAL_FRAME_ID));
    this->get_parameter_or("gyro_optical_frame_id", _optical_frame_id[GYRO],
      std::string(DEFAULT_GYRO_OPTICAL_FRAME_ID));
    this->get_parameter_or("accel_optical_frame_id", _optical_frame_id[ACCEL],
      std::string(DEFAULT_ACCEL_OPTICAL_FRAME_ID));
  }

  void setupDevice()
  {
    RCLCPP_INFO(logger_, "setupDevice...");
    try {
      _ctx.reset(new rs2::context());

      auto list = _ctx->query_devices();
      if (0 == list.size()) {
        _ctx.reset();
        RCLCPP_ERROR(logger_, "No RealSense devices were found! Terminate RealSense Node...");
        rclcpp::shutdown();
        exit(1);
      }

      // Take the first device in the list.
      // Add an ability to get the specific device to work with from outside.
      _dev = list.front();
      _ctx->set_devices_changed_callback([this](rs2::event_information & info)
        {
          if (info.was_removed(_dev)) {
            RCLCPP_FATAL(logger_, "The device has been disconnected! Terminate RealSense Node...");
            rclcpp::shutdown();
            exit(1);
          }
        });

      auto camera_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
      RCLCPP_INFO(logger_, "Device Name: %s", camera_name);

      _serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      RCLCPP_INFO(logger_, "Device Serial No: %s", _serial_no.c_str());

      auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
      RCLCPP_INFO(logger_, "Device FW version: %s", fw_ver);

      auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
      RCLCPP_INFO(logger_, "Device Product ID: %s", pid);

      RCLCPP_INFO(logger_, "Sync Mode: %s", ((_sync_frames) ? "On" : "Off"));

      auto dev_sensors = _dev.query_sensors();

      RCLCPP_INFO(logger_, "Device Sensors: ");
      for (auto && elem : dev_sensors) {
        std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
        if ("Stereo Module" == module_name || "Coded-Light Depth Sensor" == module_name) {
          auto sen = new rs2::sensor(elem);
          _sensors[DEPTH] = std::unique_ptr<rs2::sensor>(sen);
          _sensors[INFRA1] = std::unique_ptr<rs2::sensor>(sen);
          _sensors[INFRA2] = std::unique_ptr<rs2::sensor>(sen);
        } else if ("RGB Camera" == module_name) {
          _sensors[COLOR] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
        } else if ("Wide FOV Camera" == module_name) {
          _sensors[FISHEYE] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
        } else if ("Motion Module" == module_name) {
          auto hid_sensor = new rs2::sensor(elem);
          _sensors[GYRO] = std::unique_ptr<rs2::sensor>(hid_sensor);
          _sensors[ACCEL] = std::unique_ptr<rs2::sensor>(hid_sensor);
        } else {
          RCLCPP_ERROR(logger_,
            "Module Name \"%s\" isn't supported by LibRealSense! Terminate RealSense Node...",
            module_name);
          rclcpp::shutdown();
          exit(1);
        }
        RCLCPP_INFO(logger_, "%s was found.", std::string(elem.get_info(
            RS2_CAMERA_INFO_NAME)).c_str());
      }

      // Update "enable" map
      std::vector<std::vector<stream_index_pair>> streams(IMAGE_STREAMS);
      streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
      for (auto & elem : streams) {
        for (auto & stream_index : elem) {
          if (true == _enable[stream_index] && _sensors.find(stream_index) == _sensors.end()) {
            // check if device supports the enabled stream
            RCLCPP_INFO(logger_, "%s sensor isn't supported by current device! -- Skipping...",
              rs2_stream_to_string(stream_index.first));
            _enable[stream_index] = false;
          }
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(logger_, "An exception has been thrown: %s", ex.what());
      throw;
    } catch (...) {
      RCLCPP_ERROR(logger_, "Unknown exception has occured!");
      throw;
    }
  }

  void setupPublishers()
  {
    RCLCPP_INFO(logger_, "setupPublishers...");

    if (true == _enable[DEPTH]) {
      _image_publishers[DEPTH] = image_transport::create_publisher(
        this, "camera/depth/image_rect_raw");
      _info_publisher[DEPTH] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/depth/camera_info", 1);

      if (_pointcloud) {
        _pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "camera/depth/color/points", 1);
      }

      if (_align_depth) {
        _align_depth_publisher = image_transport::create_publisher(
          this, "camera/aligned_depth_to_color/image_raw");
        _align_depth_camera_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(
          "camera/aligned_depth_to_color/camera_info", 1);
      }

      if (_align_pointcloud && _align_depth) {
        _align_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "camera/aligned_depth_to_color/color/points", 1);
      }
    }

    if (true == _enable[INFRA1]) {
      _image_publishers[INFRA1] = image_transport::create_publisher(
        this, "camera/infra1/image_rect_raw");
      _info_publisher[INFRA1] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/infra1/camera_info", 1);
    }

    if (true == _enable[INFRA2]) {
      _image_publishers[INFRA2] = image_transport::create_publisher(
        this, "camera/infra2/image_rect_raw");
      _info_publisher[INFRA2] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/infra2/camera_info", 1);
    }

    if (true == _enable[COLOR]) {
      _image_publishers[COLOR] = image_transport::create_publisher(
        this, "camera/color/image_raw");
      _info_publisher[COLOR] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/color/camera_info", 1);
    }

    if (true == _enable[FISHEYE] &&
      true == _enable[DEPTH])
    {
      _image_publishers[FISHEYE] = image_transport::create_publisher(
        this, "camera/fisheye/image_raw");
      _info_publisher[FISHEYE] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/fisheye/camera_info", 1);
      _fe_to_depth_publisher = this->create_publisher<realsense_camera_msgs::msg::Extrinsics>(
        "camera/extrinsics/fisheye2depth", qos);
    }

    if (true == _enable[GYRO]) {
      _imu_publishers[GYRO] = this->create_publisher<sensor_msgs::msg::Imu>("camera/gyro/sample",
          100);
      _imu_info_publisher[GYRO] = this->create_publisher<realsense_camera_msgs::msg::IMUInfo>(
        "camera/gyro/imu_info", qos);
    }

    if (true == _enable[ACCEL]) {
      _imu_publishers[ACCEL] = this->create_publisher<sensor_msgs::msg::Imu>("camera/accel/sample",
          100);
      _imu_info_publisher[ACCEL] = this->create_publisher<realsense_camera_msgs::msg::IMUInfo>(
        "camera/accel/imu_info", qos);
    }

    if (true == _enable[FISHEYE] &&
      (true == _enable[GYRO] ||
      true == _enable[ACCEL]))
    {
      _fe_to_imu_publisher = this->create_publisher<realsense_camera_msgs::msg::Extrinsics>(
        "camera/extrinsics/fisheye2imu", qos);
    }
    _static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
  }

  void setupStreams()
  {
    RCLCPP_INFO(logger_, "setupStreams...");
    try {
      for (auto & streams : IMAGE_STREAMS) {
        for (auto & elem : streams) {
          if (true == _enable[elem]) {
            auto & sens = _sensors[elem];
            auto profiles = sens->get_stream_profiles();
            for (auto & profile : profiles) {
              auto video_profile = profile.as<rs2::video_stream_profile>();
              if (video_profile.format() == _format[elem] &&
                video_profile.width() == _width[elem] &&
                video_profile.height() == _height[elem] &&
                video_profile.fps() == _fps[elem] &&
                video_profile.stream_index() == elem.second)
              {
                _enabled_profiles[elem].push_back(profile);

                _image[elem] =
                  cv::Mat(_width[elem], _height[elem], _image_format[elem], cv::Scalar(0, 0, 0));
                RCLCPP_INFO(logger_, "%s stream is enabled - width: %d, height: %d, fps: %d",
                  _stream_name[elem].c_str(), _width[elem], _height[elem], _fps[elem]);
                break;
              }
            }
            if (_enabled_profiles.find(elem) == _enabled_profiles.end()) {
              RCLCPP_WARN(logger_, "Given stream configuration is not supported by the device!");
              RCLCPP_WARN(logger_, "Stream: %s, Format: %d, Width: %d, Height: %d, FPS: %d",
                rs2_stream_to_string(elem.first), _format[elem], _width[elem], _height[elem],
                _fps[elem]);
              _enable[elem] = false;
            }
          }
        }
      }

      // Publish image stream info
      for (auto & profiles : _enabled_profiles) {
        for (auto & profile : profiles.second) {
          auto video_profile = profile.as<rs2::video_stream_profile>();
          updateStreamCalibData(video_profile);
        }
      }

      auto frame_callback = [this](rs2::frame frame)
        {
          // We compute a ROS timestamp which is based on an initial ROS time at point of first
          // frame, and the incremental timestamp from the camera.
          // In sync mode the timestamp is based on ROS time
          if (false == _intialize_time_base) {
            _intialize_time_base = true;
            _ros_time_base = _ros_clock.now();
            _camera_time_base = frame.get_timestamp();
          }

          rclcpp::Time t;
          if (_sync_frames) {
            t = _ros_clock.now();
          } else {
            uint64_t elapsed_camera_ns = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) *
              1000000;
            t = rclcpp::Time(_ros_time_base.nanoseconds() + elapsed_camera_ns, RCL_ROS_TIME);
          }
          auto is_color_frame_arrived = false;
          auto is_depth_frame_arrived = false;
          rs2::frame depth_frame;
          if (frame.is<rs2::frameset>()) {
            RCLCPP_DEBUG(logger_, "Frameset arrived");
            auto frameset = frame.as<rs2::frameset>();
            for (auto it = frameset.begin(); it != frameset.end(); ++it) {
              auto f = (*it);
              auto stream_type = f.get_profile().stream_type();
              if (RS2_STREAM_COLOR == stream_type) {
                is_color_frame_arrived = true;
              } else if (RS2_STREAM_DEPTH == stream_type) {
                depth_frame = f;
                is_depth_frame_arrived = true;
              }

              RCLCPP_DEBUG(logger_,
                "Frameset contain %s frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                rs2_stream_to_string(stream_type), frame.get_frame_number(),
                frame.get_timestamp(), t.nanoseconds());
              publishFrame(f, t);
            }

            if (_align_depth && is_depth_frame_arrived && is_color_frame_arrived) {
              RCLCPP_DEBUG(logger_, "publishAlignedDepthTopic(...)");
              publishAlignedDepthImg(frame, t);
            }

            if (_pointcloud && is_depth_frame_arrived && is_color_frame_arrived) {
              RCLCPP_DEBUG(logger_, "publishPCTopic(...)");
              publishPCTopic(t);
            }

            if (_align_depth && _align_pointcloud && is_depth_frame_arrived &&
              is_color_frame_arrived)
            {
              RCLCPP_DEBUG(logger_, "publishAlignedPCTopic(...)");
              publishAlignedPCTopic(t);
            }

          } else {
            auto stream_type = frame.get_profile().stream_type();
            RCLCPP_DEBUG(logger_,
              "%s video frame arrived. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
              rs2_stream_to_string(stream_type), frame.get_frame_number(),
              frame.get_timestamp(), t.nanoseconds());
            publishFrame(frame, t);
          }
        };

      // Streaming IMAGES
      for (auto & streams : IMAGE_STREAMS) {
        std::vector<rs2::stream_profile> profiles;
        for (auto & elem : streams) {
          if (!_enabled_profiles[elem].empty()) {
            profiles.insert(profiles.begin(),
              _enabled_profiles[elem].begin(),
              _enabled_profiles[elem].end());
          }
        }

        if (!profiles.empty()) {
          auto stream = streams.front();
          auto & sens = _sensors[stream];
          sens->open(profiles);

          if (DEPTH == stream) {
            auto depth_sensor = sens->as<rs2::depth_sensor>();
            _depth_scale_meters = depth_sensor.get_depth_scale();
          }

          if (_sync_frames) {
            sens->start(_syncer);
          } else {
            sens->start(frame_callback);
          }
        }
      }          // end for

      if (_sync_frames) {
        _syncer.start(frame_callback);
      }

      // Streaming HID
      for (const auto streams : HID_STREAMS) {
        for (auto & elem : streams) {
          if (true == _enable[elem]) {
            auto & sens = _sensors[elem];
            auto profiles = sens->get_stream_profiles();
            for (rs2::stream_profile & profile : profiles) {
              if (profile.fps() == _fps[elem] &&
                profile.format() == _format[elem])
              {
                _enabled_profiles[elem].push_back(profile);
                break;
              }
            }
          }
        }
      }

      auto gyro_profile = _enabled_profiles.find(GYRO);
      auto accel_profile = _enabled_profiles.find(ACCEL);

      if (gyro_profile != _enabled_profiles.end() &&
        accel_profile != _enabled_profiles.end())
      {
        std::vector<rs2::stream_profile> profiles;
        profiles.insert(profiles.begin(), gyro_profile->second.begin(), gyro_profile->second.end());
        profiles.insert(profiles.begin(), accel_profile->second.begin(),
          accel_profile->second.end());
        auto & sens = _sensors[GYRO];
        sens->open(profiles);

        sens->start([this](rs2::frame frame) {
            auto stream = frame.get_profile().stream_type();
            if (false == _intialize_time_base) {
              return;
            }

            RCLCPP_DEBUG(logger_, "Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
            rs2_stream_to_string(frame.get_profile().stream_type()),
            frame.get_profile().stream_index(),
            rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

            auto stream_index = (stream == GYRO.first) ? GYRO : ACCEL;
            // if (0 != _info_publisher[stream_index].getNumSubscribers() ||
            //    0 != _imu_publishers[stream_index].getNumSubscribers())
            {
              uint64_t elapsed_camera_ns = (frame.get_timestamp() - _camera_time_base) * 1000000;
              rclcpp::Time t(_ros_time_base.nanoseconds() + elapsed_camera_ns, RCL_ROS_TIME);

              auto imu_msg = sensor_msgs::msg::Imu();
              imu_msg.header.frame_id = _optical_frame_id[stream_index];
              imu_msg.orientation.x = 0.0;
              imu_msg.orientation.y = 0.0;
              imu_msg.orientation.z = 0.0;
              imu_msg.orientation.w = 0.0;
              imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

              auto axes = *(reinterpret_cast<const float3 *>(frame.get_data()));
              if (GYRO == stream_index) {
                imu_msg.angular_velocity.x = axes.x;
                imu_msg.angular_velocity.y = axes.y;
                imu_msg.angular_velocity.z = axes.z;
              } else if (ACCEL == stream_index) {
                imu_msg.linear_acceleration.x = axes.x;
                imu_msg.linear_acceleration.y = axes.y;
                imu_msg.linear_acceleration.z = axes.z;
              }
              _seq[stream_index] += 1;
              imu_msg.header.stamp = t;
              _imu_publishers[stream_index]->publish(imu_msg);
              RCLCPP_DEBUG(logger_, "Publish %s stream", rs2_stream_to_string(
                frame.get_profile().stream_type()));
            }
          });

        if (true == _enable[GYRO]) {
          RCLCPP_INFO(logger_, "%s stream is enabled - fps: %d", _stream_name[GYRO].c_str(),
            _fps[GYRO]);
          auto gyroInfo = getImuInfo(GYRO);
          _imu_info_publisher[GYRO]->publish(gyroInfo);
        }

        if (true == _enable[ACCEL]) {
          RCLCPP_INFO(logger_, "%s stream is enabled - fps: %d",
            _stream_name[ACCEL].c_str(), _fps[ACCEL]);
          auto accelInfo = getImuInfo(ACCEL);
          _imu_info_publisher[ACCEL]->publish(accelInfo);
        }
      }


      if (true == _enable[DEPTH] &&
        true == _enable[FISHEYE])
      {
        auto ex = getFisheye2DepthExtrinsicsMsg();
        _fe_to_depth_publisher->publish(ex);
      }

      if (true == _enable[FISHEYE] &&
        (_enable[GYRO] || _enable[ACCEL]))
      {
        auto ex = getFisheye2ImuExtrinsicsMsg();
        _fe_to_imu_publisher->publish(ex);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(logger_, "An exception has been thrown: %s", ex.what());
      throw;
    } catch (...) {
      RCLCPP_ERROR(logger_, "Unknown exception has occured!");
      throw;
    }
  }

  void updateStreamCalibData(const rs2::video_stream_profile & video_profile)
  {
    stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
    auto intrinsic = video_profile.get_intrinsics();
    _stream_intrinsics[stream_index] = intrinsic;

    _camera_info[stream_index].width = intrinsic.width;
    _camera_info[stream_index].height = intrinsic.height;
    _camera_info[stream_index].header.frame_id = _optical_frame_id[stream_index];

    _camera_info[stream_index].k.at(0) = intrinsic.fx;
    _camera_info[stream_index].k.at(2) = intrinsic.ppx;
    _camera_info[stream_index].k.at(4) = intrinsic.fy;
    _camera_info[stream_index].k.at(5) = intrinsic.ppy;
    _camera_info[stream_index].k.at(8) = 1;

    _camera_info[stream_index].p.at(0) = _camera_info[stream_index].k.at(0);
    _camera_info[stream_index].p.at(1) = 0;
    _camera_info[stream_index].p.at(2) = _camera_info[stream_index].k.at(2);
    _camera_info[stream_index].p.at(3) = 0;
    _camera_info[stream_index].p.at(4) = 0;
    _camera_info[stream_index].p.at(5) = _camera_info[stream_index].k.at(4);
    _camera_info[stream_index].p.at(6) = _camera_info[stream_index].k.at(5);
    _camera_info[stream_index].p.at(7) = 0;
    _camera_info[stream_index].p.at(8) = 0;
    _camera_info[stream_index].p.at(9) = 0;
    _camera_info[stream_index].p.at(10) = 1;
    _camera_info[stream_index].p.at(11) = 0;


    // Why Depth to Color?
    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR]) {
      rs2::stream_profile depth_profile;
      if (!getEnabledProfile(DEPTH, depth_profile)) {
        RCLCPP_ERROR(logger_, "Depth profile not found!");
        rclcpp::shutdown();
        exit(1);
      }
      _depth2color_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[COLOR].front());
      // set depth to color translation values in Projection matrix (P)
      _camera_info[stream_index].p.at(3) = _depth2color_extrinsics.translation[0];             // Tx
      _camera_info[stream_index].p.at(7) = _depth2color_extrinsics.translation[1];             // Ty
      _camera_info[stream_index].p.at(11) = _depth2color_extrinsics.translation[2];            // Tz
    }

    _camera_info[stream_index].distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    _camera_info[stream_index].r.at(0) = 1.0;
    _camera_info[stream_index].r.at(1) = 0.0;
    _camera_info[stream_index].r.at(2) = 0.0;
    _camera_info[stream_index].r.at(3) = 0.0;
    _camera_info[stream_index].r.at(4) = 1.0;
    _camera_info[stream_index].r.at(5) = 0.0;
    _camera_info[stream_index].r.at(6) = 0.0;
    _camera_info[stream_index].r.at(7) = 0.0;
    _camera_info[stream_index].r.at(8) = 1.0;

    for (int i = 0; i < 5; i++) {
      _camera_info[stream_index].d.push_back(intrinsic.coeffs[i]);
    }
  }

  Eigen::Quaternionf rotationMatrixToQuaternion(float rotation[9]) const
  {
    Eigen::Matrix3f m;
    m << rotation[0], rotation[1], rotation[2],
      rotation[3], rotation[4], rotation[5],
      rotation[6], rotation[7], rotation[8];
    Eigen::Quaternionf q(m);
    return q;
  }

  void publishStaticTransforms()
  {
    RCLCPP_DEBUG(logger_, "publishStaticTransforms...");
    // Publish transforms for the cameras
    tf2::Quaternion q_c2co;
    geometry_msgs::msg::TransformStamped b2c_msg;         // Base to Color
    geometry_msgs::msg::TransformStamped c2co_msg;         // Color to Color_Optical

    tf2::Quaternion q_d2do;
    geometry_msgs::msg::TransformStamped b2d_msg;         // Base to Depth
    geometry_msgs::msg::TransformStamped d2do_msg;         // Depth to Depth_Optical

    tf2::Quaternion ir1_2_ir1o;
    geometry_msgs::msg::TransformStamped b2ir1_msg;         // Base to IR1
    geometry_msgs::msg::TransformStamped ir1_2_ir1o_msg;         // IR1 to IR1_Optical

    tf2::Quaternion ir2_2_ir2o;
    geometry_msgs::msg::TransformStamped b2ir2_msg;         // Base to IR2
    geometry_msgs::msg::TransformStamped ir2_2_ir2o_msg;         // IR2 to IR2_Optical

    // Get the current timestamp for all static transforms
    rclcpp::Time transform_ts_ = _ros_clock.now();

    // The depth frame is used as the base frame.
    // Hence no additional transformation is done from base frame to depth frame.
    // Transform base frame to depth frame
    b2d_msg.header.stamp = transform_ts_;
    b2d_msg.header.frame_id = _base_frame_id;
    b2d_msg.child_frame_id = _frame_id[DEPTH];
    b2d_msg.transform.translation.x = 0;
    b2d_msg.transform.translation.y = 0;
    b2d_msg.transform.translation.z = 0;
    b2d_msg.transform.rotation.x = 0;
    b2d_msg.transform.rotation.y = 0;
    b2d_msg.transform.rotation.z = 0;
    b2d_msg.transform.rotation.w = 1;
    _static_tf_broadcaster_->sendTransform(b2d_msg);

    // Transform depth frame to depth optical frame
    q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    d2do_msg.header.stamp = transform_ts_;
    d2do_msg.header.frame_id = _frame_id[DEPTH];
    d2do_msg.child_frame_id = _optical_frame_id[DEPTH];
    d2do_msg.transform.translation.x = 0;
    d2do_msg.transform.translation.y = 0;
    d2do_msg.transform.translation.z = 0;
    d2do_msg.transform.rotation.x = q_d2do.getX();
    d2do_msg.transform.rotation.y = q_d2do.getY();
    d2do_msg.transform.rotation.z = q_d2do.getZ();
    d2do_msg.transform.rotation.w = q_d2do.getW();
    _static_tf_broadcaster_->sendTransform(d2do_msg);


    if (true == _enable[COLOR]) {
      // Transform base frame to color frame
      auto q = rotationMatrixToQuaternion(_depth2color_extrinsics.rotation);

      b2c_msg.header.stamp = transform_ts_;
      b2c_msg.header.frame_id = _base_frame_id;
      b2c_msg.child_frame_id = _frame_id[COLOR];
      b2c_msg.transform.translation.x = _depth2color_extrinsics.translation[2];
      b2c_msg.transform.translation.y = -_depth2color_extrinsics.translation[0];
      b2c_msg.transform.translation.z = -_depth2color_extrinsics.translation[1];
      b2c_msg.transform.rotation.x = q.x();
      b2c_msg.transform.rotation.y = q.y();
      b2c_msg.transform.rotation.z = q.z();
      b2c_msg.transform.rotation.w = q.w();
      _static_tf_broadcaster_->sendTransform(b2c_msg);

      // Transform color frame to color optical frame
      q_c2co.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
      c2co_msg.header.stamp = transform_ts_;
      c2co_msg.header.frame_id = _frame_id[COLOR];
      c2co_msg.child_frame_id = _optical_frame_id[COLOR];
      c2co_msg.transform.translation.x = 0;
      c2co_msg.transform.translation.y = 0;
      c2co_msg.transform.translation.z = 0;
      c2co_msg.transform.rotation.x = q_c2co.getX();
      c2co_msg.transform.rotation.y = q_c2co.getY();
      c2co_msg.transform.rotation.z = q_c2co.getZ();
      c2co_msg.transform.rotation.w = q_c2co.getW();
      _static_tf_broadcaster_->sendTransform(c2co_msg);
    }

    if (_enable[DEPTH]) {
      rs2::stream_profile depth_profile;
      if (!getEnabledProfile(DEPTH, depth_profile)) {
        RCLCPP_ERROR(logger_, "Depth profile not found!");
        rclcpp::shutdown();
        exit(1);
      }
      if (true == _enable[INFRA1]) {
        auto d2ir1_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[INFRA1].front());
        auto q = rotationMatrixToQuaternion(d2ir1_extrinsics.rotation);

        // Transform base frame to infra1
        b2ir1_msg.header.stamp = transform_ts_;
        b2ir1_msg.header.frame_id = _base_frame_id;
        b2ir1_msg.child_frame_id = _frame_id[INFRA1];
        b2ir1_msg.transform.translation.x = d2ir1_extrinsics.translation[2];
        b2ir1_msg.transform.translation.y = -d2ir1_extrinsics.translation[0];
        b2ir1_msg.transform.translation.z = -d2ir1_extrinsics.translation[1];

        b2ir1_msg.transform.rotation.x = q.x();
        b2ir1_msg.transform.rotation.y = q.y();
        b2ir1_msg.transform.rotation.z = q.z();
        b2ir1_msg.transform.rotation.w = q.w();
        _static_tf_broadcaster_->sendTransform(b2ir1_msg);

        // Transform infra1 frame to infra1 optical frame
        ir1_2_ir1o.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
        ir1_2_ir1o_msg.header.stamp = transform_ts_;
        ir1_2_ir1o_msg.header.frame_id = _frame_id[INFRA1];
        ir1_2_ir1o_msg.child_frame_id = _optical_frame_id[INFRA1];
        ir1_2_ir1o_msg.transform.translation.x = 0;
        ir1_2_ir1o_msg.transform.translation.y = 0;
        ir1_2_ir1o_msg.transform.translation.z = 0;
        ir1_2_ir1o_msg.transform.rotation.x = ir1_2_ir1o.getX();
        ir1_2_ir1o_msg.transform.rotation.y = ir1_2_ir1o.getY();
        ir1_2_ir1o_msg.transform.rotation.z = ir1_2_ir1o.getZ();
        ir1_2_ir1o_msg.transform.rotation.w = ir1_2_ir1o.getW();
        _static_tf_broadcaster_->sendTransform(ir1_2_ir1o_msg);
      }

      if (true == _enable[INFRA2]) {
        auto d2ir2_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[INFRA2].front());
        auto q = rotationMatrixToQuaternion(d2ir2_extrinsics.rotation);

        // Transform base frame to infra2
        b2ir2_msg.header.stamp = transform_ts_;
        b2ir2_msg.header.frame_id = _base_frame_id;
        b2ir2_msg.child_frame_id = _frame_id[INFRA2];
        b2ir2_msg.transform.translation.x = d2ir2_extrinsics.translation[2];
        b2ir2_msg.transform.translation.y = -d2ir2_extrinsics.translation[0];
        b2ir2_msg.transform.translation.z = -d2ir2_extrinsics.translation[1];
        b2ir2_msg.transform.rotation.x = q.x();
        b2ir2_msg.transform.rotation.y = q.y();
        b2ir2_msg.transform.rotation.z = q.z();
        b2ir2_msg.transform.rotation.w = q.w();
        _static_tf_broadcaster_->sendTransform(b2ir2_msg);

        // Transform infra2 frame to infra1 optical frame
        ir2_2_ir2o.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
        ir2_2_ir2o_msg.header.stamp = transform_ts_;
        ir2_2_ir2o_msg.header.frame_id = _frame_id[INFRA2];
        ir2_2_ir2o_msg.child_frame_id = _optical_frame_id[INFRA2];
        ir2_2_ir2o_msg.transform.translation.x = 0;
        ir2_2_ir2o_msg.transform.translation.y = 0;
        ir2_2_ir2o_msg.transform.translation.z = 0;
        ir2_2_ir2o_msg.transform.rotation.x = ir2_2_ir2o.getX();
        ir2_2_ir2o_msg.transform.rotation.y = ir2_2_ir2o.getY();
        ir2_2_ir2o_msg.transform.rotation.z = ir2_2_ir2o.getZ();
        ir2_2_ir2o_msg.transform.rotation.w = ir2_2_ir2o.getW();
        _static_tf_broadcaster_->sendTransform(ir2_2_ir2o_msg);
      }
    }
    // Publish Fisheye TF
  }

  void alignFrame(
    const rs2_intrinsics & from_intrin,
    const rs2_intrinsics & other_intrin,
    rs2::frame from_image,
    uint32_t output_image_bytes_per_pixel,
    const rs2_extrinsics & from_to_other,
    std::vector<uint8_t> & out_vec)
  {
    static const auto meter_to_mm = 0.001f;
    uint8_t * p_out_frame = out_vec.data();

    static const auto blank_color = 0x00;
    memset(p_out_frame, blank_color,
      other_intrin.height * other_intrin.width * output_image_bytes_per_pixel);

    auto p_from_frame = reinterpret_cast<const uint8_t *>(from_image.get_data());
    auto from_stream_type = from_image.get_profile().stream_type();
    float depth_units = ((from_stream_type == RS2_STREAM_DEPTH) ? _depth_scale_meters : 1.f);
    for (int from_y = 0; from_y < from_intrin.height; ++from_y) {
      int from_pixel_index = from_y * from_intrin.width;
      for (int from_x = 0; from_x < from_intrin.width; ++from_x, ++from_pixel_index) {
        // Skip over depth pixels with the value of zero
        float depth =
          (from_stream_type ==
          RS2_STREAM_DEPTH) ? (depth_units *
          ((const uint16_t *)p_from_frame)[from_pixel_index]) : 1.f;
        if (depth) {
          // Map the top-left corner of the depth pixel onto the other image
          float from_pixel[2] = {from_x - 0.5f, from_y - 0.5f}, from_point[3], other_point[3],
            other_pixel[2];
          rs2_deproject_pixel_to_point(from_point, &from_intrin, from_pixel, depth);
          rs2_transform_point_to_point(other_point, &from_to_other, from_point);
          rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
          const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
          const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

          // Map the bottom-right corner of the depth pixel onto the other image
          from_pixel[0] = from_x + 0.5f; from_pixel[1] = from_y + 0.5f;
          rs2_deproject_pixel_to_point(from_point, &from_intrin, from_pixel, depth);
          rs2_transform_point_to_point(other_point, &from_to_other, from_point);
          rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
          const int other_x1 = static_cast<int>(other_pixel[0] + 0.5f);
          const int other_y1 = static_cast<int>(other_pixel[1] + 0.5f);

          if (other_x0 < 0 || other_y0 < 0 || other_x1 >= other_intrin.width ||
            other_y1 >= other_intrin.height)
          {
            continue;
          }

          for (int y = other_y0; y <= other_y1; ++y) {
            for (int x = other_x0; x <= other_x1; ++x) {
              int out_pixel_index = y * other_intrin.width + x;
              auto p_from_depth_frame = reinterpret_cast<const uint16_t *>(p_from_frame);
              auto p_out_depth_frame = reinterpret_cast<uint16_t *>(p_out_frame);
              p_out_depth_frame[out_pixel_index] = p_from_depth_frame[from_pixel_index] *
                (  depth_units / meter_to_mm);
            }
          }
        }
      }
    }
  }

  rs2_extrinsics getRsExtrinsics(
    const stream_index_pair & from_stream,
    const stream_index_pair & to_stream)
  {
    auto & from = _enabled_profiles[from_stream].front();
    auto & to = _enabled_profiles[to_stream].front();
    return from.get_extrinsics_to(to);
  }

  void publishAlignedDepthImg(rs2::frame depth_frame, const rclcpp::Time & t)
  {
    rs2::align align(RS2_STREAM_COLOR);
    _aligned_frameset = depth_frame.apply_filter(align);
    rs2::depth_frame aligned_depth = _aligned_frameset.get_depth_frame();
    RCLCPP_DEBUG(logger_, "aligned done!");

    auto vf = aligned_depth.as<rs2::video_frame>();
    auto depth_image = cv::Mat(cv::Size(vf.get_width(), vf.get_height()), _image_format[DEPTH],
        const_cast<void *>(vf.get_data()), cv::Mat::AUTO_STEP);

    sensor_msgs::msg::Image::SharedPtr img;
    auto info_msg = _camera_info[DEPTH];
    img = cv_bridge::CvImage(
      std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_image).toImageMsg();
    auto image = aligned_depth.as<rs2::video_frame>();
    auto bpp = image.get_bytes_per_pixel();
    auto height = image.get_height();
    auto width = image.get_width();
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->step = width * bpp;
    img->header.frame_id = _optical_frame_id[COLOR];
    img->header.stamp = t;
    _align_depth_publisher.publish(img);
    _align_depth_camera_publisher->publish(info_msg);
  }

  void publishPCTopic(const rclcpp::Time & t)
  {
    auto color_intrinsics = _stream_intrinsics[COLOR];
    auto image_depth16 = reinterpret_cast<const uint16_t *>(_image[DEPTH].data);
    auto depth_intrinsics = _stream_intrinsics[DEPTH];
    sensor_msgs::msg::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH];
    msg_pointcloud.width = depth_intrinsics.width;
    msg_pointcloud.height = depth_intrinsics.height;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg_pointcloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg_pointcloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg_pointcloud, "b");

    float depth_point[3], color_point[3], color_pixel[2], scaled_depth;
    unsigned char * color_data = _image[COLOR].data;

    // Fill the PointCloud2 fields
    for (int y = 0; y < depth_intrinsics.height; ++y) {
      for (int x = 0; x < depth_intrinsics.width; ++x) {
        scaled_depth = static_cast<float>(*image_depth16) * _depth_scale_meters;
        float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
        rs2_deproject_pixel_to_point(depth_point, &depth_intrinsics, depth_pixel, scaled_depth);

        if (depth_point[2] <= 0.f || depth_point[2] > 5.f) {
          depth_point[0] = 0.f;
          depth_point[1] = 0.f;
          depth_point[2] = 0.f;
        }

        *iter_x = depth_point[0];
        *iter_y = depth_point[1];
        *iter_z = depth_point[2];

        rs2_transform_point_to_point(color_point, &_depth2color_extrinsics, depth_point);
        rs2_project_point_to_pixel(color_pixel, &color_intrinsics, color_point);

        if (color_pixel[1] < 0.f || color_pixel[1] > color_intrinsics.height ||
          color_pixel[0] < 0.f || color_pixel[0] > color_intrinsics.width)
        {
          // For out of bounds color data, default to a shade of blue in order to visually
          // distinguish holes. This color value is same as the librealsense out of bounds color
          // value.
          *iter_r = static_cast<uint8_t>(96);
          *iter_g = static_cast<uint8_t>(157);
          *iter_b = static_cast<uint8_t>(198);
        } else {
          auto i = static_cast<int>(color_pixel[0]);
          auto j = static_cast<int>(color_pixel[1]);

          auto offset = i * 3 + j * color_intrinsics.width * 3;
          *iter_r = static_cast<uint8_t>(color_data[offset]);
          *iter_g = static_cast<uint8_t>(color_data[offset + 1]);
          *iter_b = static_cast<uint8_t>(color_data[offset + 2]);
        }

        ++image_depth16;
        ++iter_x; ++iter_y; ++iter_z;
        ++iter_r; ++iter_g; ++iter_b;
      }
    }
    _pointcloud_publisher->publish(msg_pointcloud);
  }

  void publishAlignedPCTopic(const rclcpp::Time & t)
  {
    rs2::depth_frame aligned_depth = _aligned_frameset.get_depth_frame();

    auto image_depth16 = reinterpret_cast<const uint16_t *>(aligned_depth.get_data());
    auto depth_intrinsics = _stream_intrinsics[COLOR];
    unsigned char * color_data = _image[COLOR].data;
    sensor_msgs::msg::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = _optical_frame_id[COLOR];
    msg_pointcloud.width = depth_intrinsics.width;
    msg_pointcloud.height = depth_intrinsics.height;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

    modifier.setPointCloud2Fields(3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg_pointcloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg_pointcloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg_pointcloud, "b");

    float std_nan = std::numeric_limits<float>::quiet_NaN();
    float depth_point[3], scaled_depth;

    // Fill the PointCloud2 fields
    for (int y = 0; y < depth_intrinsics.height; ++y) {
      for (int x = 0; x < depth_intrinsics.width; ++x) {
        scaled_depth = static_cast<float>(*image_depth16) * _depth_scale_meters;
        float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
        rs2_deproject_pixel_to_point(depth_point, &depth_intrinsics, depth_pixel, scaled_depth);
        auto iter_offset = x + y * depth_intrinsics.width;

        if (depth_point[2] <= 0.f || depth_point[2] > 5.f) {
          *(iter_x + iter_offset) = std_nan;
          *(iter_y + iter_offset) = std_nan;
          *(iter_z + iter_offset) = std_nan;
          *(iter_r + iter_offset) = static_cast<uint8_t>(96);
          *(iter_g + iter_offset) = static_cast<uint8_t>(157);
          *(iter_b + iter_offset) = static_cast<uint8_t>(198);
        } else {
          *(iter_x + iter_offset) = depth_point[0];
          *(iter_y + iter_offset) = depth_point[1];
          *(iter_z + iter_offset) = depth_point[2];
          *(iter_r + iter_offset) = color_data[iter_offset * 3];
          *(iter_g + iter_offset) = color_data[iter_offset * 3 + 1];
          *(iter_b + iter_offset) = color_data[iter_offset * 3 + 2];
        }

        ++image_depth16;
      }
    }

    _align_pointcloud_publisher->publish(msg_pointcloud);
  }


  Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics & extrinsics) const
  {
    Extrinsics extrinsicsMsg;
    for (int i = 0; i < 9; ++i) {
      extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
      if (i < 3) {
        extrinsicsMsg.translation[i] = extrinsics.translation[i];
      }
    }

    return extrinsicsMsg;
  }

  Extrinsics getFisheye2ImuExtrinsicsMsg()
  {
    auto & fisheye = _enabled_profiles[FISHEYE].front();
    auto & hid = _enabled_profiles[GYRO].front();
    Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(fisheye.get_extrinsics_to(hid));
    extrinsicsMsg.header.frame_id = "fisheye2imu_extrinsics";
    return extrinsicsMsg;
  }

  Extrinsics getFisheye2DepthExtrinsicsMsg()
  {
    auto & fisheye = _enabled_profiles[FISHEYE].front();
    auto & depth = _enabled_profiles[DEPTH].front();
    Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(fisheye.get_extrinsics_to(depth));
    extrinsicsMsg.header.frame_id = "fisheye2depth_extrinsics";
    return extrinsicsMsg;
  }

  struct float3
  {
    float x, y, z;
  };

  IMUInfo getImuInfo(const stream_index_pair & stream_index)
  {
    IMUInfo info;
    #if (RS2_API_VERSION >= 20901)
    auto sp = _enabled_profiles[stream_index].front().as<rs2::motion_stream_profile>();
    auto imuIntrinsics = sp.get_motion_intrinsics();
    #else
    auto imuIntrinsics = _sensors[stream_index]->get_motion_intrinsics(stream_index.first);
    #endif
    if (GYRO == stream_index) {
      info.header.frame_id = "imu_gyro";
    } else if (ACCEL == stream_index) {
      info.header.frame_id = "imu_accel";
    }

    auto index = 0;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        info.data[index] = imuIntrinsics.data[i][j];
        ++index;
      }
      info.noise_variances[i] = imuIntrinsics.noise_variances[i];
      info.bias_variances[i] = imuIntrinsics.bias_variances[i];
    }
    return info;
  }

  void tryGetLogSeverity(rs2_log_severity & severity) const
  {
    static const char * severity_var_name = "LRS_LOG_LEVEL";
    auto content = getenv(severity_var_name);

    if (content) {
      std::string content_str(content);
      std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

      for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++) {
        auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
        std::transform(current.begin(), current.end(), current.begin(), ::toupper);
        if (content_str == current) {
          severity = (rs2_log_severity)i;
          break;
        }
      }
    }
  }

  void publishFrame(rs2::frame f, const rclcpp::Time & t)
  {
    RCLCPP_DEBUG(logger_, "publishFrame(...)");
    stream_index_pair stream{f.get_profile().stream_type(), f.get_profile().stream_index()};
    auto & image = _image[stream];
    image.data = const_cast<uchar *>(reinterpret_cast<const uchar *>(f.get_data()));
    ++(_seq[stream]);
    auto & info_publisher = _info_publisher[stream];
    auto & image_publisher = _image_publishers[stream];
    // if (0 != info_publisher.getNumSubscribers() ||
    //     0 != image_publisher.getNumSubscribers())
    {
      auto width = 0;
      auto height = 0;
      auto bpp = 1;
      if (f.is<rs2::video_frame>()) {
        auto image = f.as<rs2::video_frame>();
        width = image.get_width();
        height = image.get_height();
        bpp = image.get_bytes_per_pixel();
      }

      sensor_msgs::msg::Image::SharedPtr img;
      img = cv_bridge::CvImage(std_msgs::msg::Header(), _encoding[stream], image).toImageMsg();
      img->width = width;
      img->height = height;
      img->is_bigendian = false;
      img->step = width * bpp;
      img->header.frame_id = _optical_frame_id[stream];
      img->header.stamp = t;

      auto & cam_info = _camera_info[stream];
      cam_info.header.stamp = t;
      info_publisher->publish(cam_info);

      image_publisher.publish(img);
      RCLCPP_DEBUG(logger_, "%s stream published",
        rs2_stream_to_string(f.get_profile().stream_type()));
    }
  }

  bool getEnabledProfile(const stream_index_pair & stream_index, rs2::stream_profile & profile)
  {
    // Assuming that all D400 SKUs have depth sensor
    auto profiles = _enabled_profiles[stream_index];
    auto it = std::find_if(profiles.begin(), profiles.end(),
        [&](const rs2::stream_profile & profile)
        {return profile.stream_type() == stream_index.first;});
    if (it == profiles.end()) {
      return false;
    }

    profile = *it;
    return true;
  }

  rclcpp::Clock _ros_clock;
  std::unique_ptr<rs2::context> _ctx;

  std::map<stream_index_pair, std::unique_ptr<rs2::sensor>> _sensors;

  std::string _serial_no;
  float _depth_scale_meters;

  std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
  std::map<stream_index_pair, int> _width;
  std::map<stream_index_pair, int> _height;
  std::map<stream_index_pair, int> _fps;
  std::map<stream_index_pair, bool> _enable;
  std::map<stream_index_pair, std::string> _stream_name;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster_;

  std::map<stream_index_pair, image_transport::Publisher> _image_publishers;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
  std::map<stream_index_pair, int> _image_format;
  std::map<stream_index_pair, rs2_format> _format;
  std::map<stream_index_pair,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _info_publisher;
  std::map<stream_index_pair,
    rclcpp::Publisher<realsense_camera_msgs::msg::IMUInfo>::SharedPtr> _imu_info_publisher;
  std::map<stream_index_pair, cv::Mat> _image;
  std::map<stream_index_pair, std::string> _encoding;

  std::string _base_frame_id;
  std::map<stream_index_pair, std::string> _frame_id;
  std::map<stream_index_pair, std::string> _optical_frame_id;
  std::map<stream_index_pair, int> _seq;
  std::map<stream_index_pair, int> _unit_step_size;
  std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _camera_info;
  rclcpp::Publisher<realsense_camera_msgs::msg::Extrinsics>::SharedPtr _fe_to_depth_publisher,
    _fe_to_imu_publisher;

  rclcpp::QoS qos;
  bool _intialize_time_base;
  double _camera_time_base;
  std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

  image_transport::Publisher _align_depth_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _align_depth_camera_publisher;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _align_pointcloud_publisher;

  rclcpp::Time _ros_time_base;
  rclcpp::Logger logger_ = rclcpp::get_logger("RealSenseCameraNode");
  rclcpp::TimerBase::SharedPtr timer_;
  bool _sync_frames;
  bool _pointcloud;
  bool _align_pointcloud;
  bool _align_depth;
  PipelineSyncer _syncer;
  rs2_extrinsics _depth2color_extrinsics;

  rs2::frameset _aligned_frameset;
};  // end class
}  // namespace realsense_ros2_camera

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<realsense_ros2_camera::RealSenseCameraNode>();
  node->onInit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
