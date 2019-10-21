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
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "opencv2/opencv.hpp"
#include "realsense/rs_factory.hpp"
#include "realsense/rs_d435.hpp"
#include "realsense/rs_d435i.hpp"
#include "realsense/rs_t265.hpp"

TEST(UnitTestFunction, testLibraryFunctions)
{
  ASSERT_NO_THROW({
    auto realsense_node = std::make_shared<realsense::RealSenseNodeFactory>();
  });
}

TEST(UnitTestFunction, testLibraryIncorrectInputs)
{
  try 
  {
    auto realsense_node = std::make_shared<realsense::RealSenseNodeFactory>("ZR300", "0");
  }
  catch (...)
  {
    SUCCEED();
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
