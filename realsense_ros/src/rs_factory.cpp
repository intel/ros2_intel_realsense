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

#include "opencv2/opencv.hpp"
#include "realsense/rs_factory.hpp"
#include "realsense/rs_d435.hpp"
#include "realsense/rs_d435i.hpp"
#include "realsense/rs_t265.hpp"

namespace realsense
{

RealSenseNodeFactory::RealSenseNodeFactory(const rclcpp::NodeOptions & node_options)
: Node("camera", "/", node_options)
{
  init();
}

RealSenseNodeFactory::RealSenseNodeFactory(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
: Node(node_name, ns, node_options)
{
  init();
}

RealSenseNodeFactory::~RealSenseNodeFactory()
{
  query_thread_.detach();
}

void RealSenseNodeFactory::init()
{
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.read_only = true;
  auto param_value = declare_parameter("serial_no");
  if (param_value.get_type() == rclcpp::PARAMETER_NOT_SET) {
    RCLCPP_INFO(this->get_logger(), "Device's serial number is not set, enabling the default device!");
  } else {
    serial_no_ = std::to_string(param_value.get<rclcpp::PARAMETER_INTEGER>());
  }
  try {
    query_thread_ = std::thread([=]()
              {
                std::chrono::milliseconds TIMESPAN(600);
                while (!dev_) {
                  auto dev_lst = ctx_.query_devices();
                  getDevice(dev_lst);
                  if (dev_) {
                      std::function<void(rs2::event_information&)> changeDeviceCallback_function = [this](rs2::event_information& info){changeDeviceCallback(info);};
                      ctx_.set_devices_changed_callback(changeDeviceCallback_function);
                    startDevice();
                  } else {
                    std::this_thread::sleep_for(TIMESPAN);
                  }
                }
              });
  } catch(const std::exception& ex) {
    RCLCPP_WARN(this->get_logger(), "An exception has been thrown: %s",ex.what());
    exit(1);
  } catch(...) {
    RCLCPP_WARN(this->get_logger(), "Unknown exception has occured!");
    exit(1);
  } 
}

void RealSenseNodeFactory::startDevice()
{
  std::string pid_str = dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
  uint16_t pid = std::stoi(pid_str, 0, 16);
    switch(pid) {
      case RS435_RGB_PID:
      case RS415_PID:
      case RS_USB2_PID:
        RCLCPP_INFO(this->get_logger(), "Create a node for D4X5 Camera");
        rs_node_ = std::make_unique<RealSenseD435>(ctx_, dev_, *this);
        break;
      case RS435i_RGB_PID:
        RCLCPP_INFO(this->get_logger(), "Create a node for D435i Camera");
        rs_node_ = std::make_unique<RealSenseD435I>(ctx_, dev_, *this);
        break;
      case RS_T265_PID:
        RCLCPP_INFO(this->get_logger(), "Create a node for T265 Camera");
        rs_node_ = std::make_unique<RealSenseT265>(ctx_, dev_, *this);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unsupported device! Product ID: 0x%s", pid_str);
        rclcpp::shutdown();
    }
  rs_node_->startPipeline();
}

void RealSenseNodeFactory::changeDeviceCallback(rs2::event_information& info)
{
  if (info.was_removed(dev_)) {
    RCLCPP_ERROR(this->get_logger(), "The device has been disconnected!");
    rs_node_.release();
    // rs_node_.reset(nullptr);
    dev_ = rs2::device();
  }
  if (!dev_) {
    rs2::device_list new_devices = info.get_new_devices();
    if (new_devices.size() > 0) {
      RCLCPP_INFO(this->get_logger(), "Checking new devices...");
      getDevice(new_devices);
      if (dev_) {
        startDevice();
      }
    }
  }
}

void RealSenseNodeFactory::getDevice(rs2::device_list & list)
{
  if (!dev_) {
    if (0 == list.size()) {
      RCLCPP_ERROR(this->get_logger(), "No RealSense devices were found!");
    }
    else {
      bool found = false;
      for (auto && dev : list) {
        auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        RCLCPP_INFO(this->get_logger(), "Device with serial number %s was found.", sn);
        if (serial_no_.empty() || sn == serial_no_) {
          dev_ = dev;
          serial_no_ = sn;
          found = true;
          break;
        }
      }
      if (!found) {
        // T265 could be caught by another node.
        RCLCPP_ERROR(this->get_logger(), "The Device with serial number %s is not found. Please make sure it is connected.", serial_no_.c_str());
      }
    }
  bool remove_tm2_handle(dev_ && RS_T265_PID != std::stoi(dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
  if (remove_tm2_handle) {
    ctx_.unload_tracking_module();
  }

  }
}
}  // namespace realsense

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(realsense::RealSenseNodeFactory)