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
RealSenseNodeFactory::RealSenseNodeFactory(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
: Node(node_name, ns, node_options)
{
  rs2::context ctx;
  rs2::device dev;
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.read_only = true;
  auto param_value = declare_parameter("serial_no", rclcpp::ParameterValue(), param_desc);
  if (param_value.get_type() == rclcpp::PARAMETER_NOT_SET) {
    dev = ctx.query_devices().front();
  } else {
    std::string serial_no = std::to_string(param_value.get<rclcpp::PARAMETER_INTEGER>());
    auto dev_list = ctx.query_devices();
    bool found = false;
    for (auto && d : dev_list) {
      if (d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial_no) {
        found = true;
        dev = d;
        break;
      }
    }
    if (found == false) {
      RCLCPP_ERROR(this->get_logger(), "The Device with serial number %s is not found. Please make sure it is connected.", serial_no.c_str());
      rclcpp::shutdown();
    }
  }
  std::string pid_str = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
  uint16_t pid = std::stoi(pid_str, 0, 16);
  switch(pid) {
    case RS415_PID:
    case RS435_RGB_PID:
      RCLCPP_INFO(this->get_logger(), "Create a node for D4X5 Camera");
      rs_node_ = std::make_shared<RealSenseD435>(ctx, dev, *this);
      rs_node_->startPipeline();
      break;
    case RS435i_RGB_PID:
      RCLCPP_INFO(this->get_logger(), "Create a node for D435i Camera");
      rs_node_ = std::make_shared<RealSenseD435I>(ctx, dev, *this);
      rs_node_->startPipeline();
      break;
    case RS_T265_PID:
      RCLCPP_INFO(this->get_logger(), "Create a node for T265 Camera");
      rs_node_ = std::make_shared<RealSenseT265>(ctx, dev, *this);
      rs_node_->startPipeline();
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unsupported device! Product ID: 0x%s", pid_str.c_str());
      rclcpp::shutdown();
  } 
}
}  // namespace realsense

//#include "rclcpp_components/register_node_macro.hpp"
//RCLCPP_COMPONENTS_REGISTER_NODE(realsense::RealSenseNodeFactory)