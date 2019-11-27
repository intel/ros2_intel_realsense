cmake_minimum_required(VERSION 3.5)
project(realsense_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Compiler Defense Flags
if(UNIX OR APPLE)
  # Linker flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security -Wall")
  # Dot not forward c++ flag to GPU beucause it is not supported
  set( CUDA_PROPAGATE_HOST_FLAGS OFF )
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_FORTIFY_SOURCE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
endif()

if(CMAKE_BUILD_TYPE EQUAL "RELEASE")
  message(STATUS "Create Release Build.")
  set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
  message(STATUS "Create Debug Build.")
endif()

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(realsense_msgs)

find_package(realsense2)
if(NOT realsense2_FOUND)
  message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${realsense2_INCLUDE_DIRS}
)

set(node_plugins "")

add_library(${PROJECT_NAME} SHARED
  src/rs_factory.cpp
  src/rs_base.cpp
  src/rs_d435.cpp
  src/rs_d435i.cpp
  src/rs_t265.cpp
)

target_link_libraries(${PROJECT_NAME}
  realsense2
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  cv_bridge
  tf2
  tf2_ros
  pcl_conversions
  realsense2
  realsense_msgs
)

rclcpp_components_register_nodes(${PROJECT_NAME} "realsense::RealSenseNodeFactory")
set(node_plugins "${node_plugins}realsense::RealSenseNodeFactory;$<TARGET_FILE:${PROJECT_NAME}>\n")

file(GENERATE
  OUTPUT
  "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/$<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
  CONTENT "${node_plugins}")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(realsense_msgs)
  find_package(realsense2)
  find_package(image_transport REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  macro(custom_gtest target)
    ament_add_gtest(${target} ${ARGN})
    if(TARGET ${target})
      target_include_directories(${target} PUBLIC
        ${${PROJECT_NAME}_INCLUDE_DIRS}
      )
      target_link_libraries(${target}
        realsense2
	${PROJECT_NAME})
      ament_target_dependencies(${target}
        "rclcpp"
        "nav_msgs"
        "sensor_msgs"
        "cv_bridge"
        "tf2"
	"tf2_ros"
        "pcl_conversions"
        "realsense_msgs"
        "realsense2"
        "image_transport")
    endif()
  endmacro()

  custom_gtest(unittest_function
    "tests/unittest_function.cpp"
    TIMEOUT 300)
  custom_gtest(camera_core
    "tests/camera_core.cpp"
    TIMEOUT 300)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
