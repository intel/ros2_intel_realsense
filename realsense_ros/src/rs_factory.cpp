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
: Node("camera", "/", node_options),logger_(rclcpp::get_logger("camera"))
{
  init();
}

RealSenseNodeFactory::RealSenseNodeFactory(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
: Node(node_name, ns, node_options), logger_(rclcpp::get_logger(node_name))
{
  init();
}

void RealSenseNodeFactory::init()
{
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.read_only = true;
  auto param_value = declare_parameter("serial_no", rclcpp::ParameterValue(), param_desc);
  serial_no_ = std::to_string(param_value.get<rclcpp::PARAMETER_INTEGER>());
  try
  {
    _query_thread = std::thread([=]()
              {
                std::chrono::milliseconds timespan(600);
                while (!dev)
                {
                  auto dev_lst = ctx.query_devices();
                  getDevice(dev_lst);
                  if (dev)
                  {
                      std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
                      ctx.set_devices_changed_callback(change_device_callback_function);
                    StartDevice();
                  }
                  else
                  {
                    std::this_thread::sleep_for(timespan);
                  }
                  
                }
              });
  }
  catch(const std::exception& ex)
  {
    RCLCPP_WARN(logger_, "An exception has been thrown: %s",ex.what());
    exit(1);
  }
  catch(...)
  {
    RCLCPP_WARN(logger_, "Unknown exception has occured!");
    exit(1);
  } 
}

RealSenseNodeFactory::~RealSenseNodeFactory()
{
  for(rs2::sensor sensor : dev.query_sensors())
  {
    sensor.stop();
    sensor.close();
  }
}

void RealSenseNodeFactory::StartDevice()
{
  std::string pid_str = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
  uint16_t pid = std::stoi(pid_str, 0, 16);
    switch(pid) {
      case RS435_RGB_PID:
        RCLCPP_INFO(logger_, "Create a node for D4X5 Camera");
        rs_node_ = std::make_unique<RealSenseD435>(ctx, dev, *this);
        break;
      case RS435i_RGB_PID:
        RCLCPP_INFO(logger_, "Create a node for D435i Camera");
        rs_node_ = std::make_unique<RealSenseD435I>(ctx, dev, *this);
        break;
      case RS_T265_PID:
        RCLCPP_INFO(logger_, "Create a node for T265 Camera");
        rs_node_ = std::make_unique<RealSenseT265>(ctx, dev, *this);
        break;
      default:
        RCLCPP_ERROR(logger_, "Unsupported device! Product ID: 0x%s", pid_str);
        rclcpp::shutdown();
    }
  rs_node_->startPipeline();
}

void RealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
  if (info.was_removed(dev))
  {
    RCLCPP_ERROR(logger_, "The device has been disconnected!");
    rs_node_.release();
    rs_node_.reset(nullptr);
    dev = rs2::device();
  }
  if (!dev)
  {
    rs2::device_list new_devices = info.get_new_devices();
    if (new_devices.size() > 0)
    {
      RCLCPP_INFO(logger_, "Checking new devices...");
      getDevice(new_devices);
      if (dev)
      {
        StartDevice();
      }
    }
  }
}

void RealSenseNodeFactory::getDevice(rs2::device_list & list)
{
  if (!dev)
  {
    if (0 == list.size())
    {
      RCLCPP_ERROR(logger_, "No RealSense devices were found!");
    }
    else
    {
      bool found = false;
      for (auto&& _dev : list)
      {
        auto sn = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        RCLCPP_INFO(logger_, "Device with serial number %s was found.", sn);
        if (serial_no_.empty() || sn == serial_no_)
        {
          dev = _dev;
          serial_no_ = sn;
          found = true;
          break;
        }
      }
      if (!found)
      {
        // T265 could be caught by another node.
        RCLCPP_ERROR(logger_, "The Device with serial number %s is not found. Please make sure it is connected.", serial_no_.c_str());
      }
    }
  bool remove_tm2_handle(dev && RS_T265_PID != std::stoi(dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
  if (remove_tm2_handle)
  {
    ctx.unload_tracking_module();
  }

  }
}
}  // namespace realsense

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(realsense::RealSenseNodeFactory)