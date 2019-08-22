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

#include "realsense/rs_factory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto t265_node = std::make_shared<realsense::RealSenseNodeFactory>("camera", "/t265");
  auto d435_node = std::make_shared<realsense::RealSenseNodeFactory>("camera", "/d435");
  
  exec.add_node(t265_node);
  exec.add_node(d435_node);
  
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
