cmake_minimum_required(VERSION 3.5)
project(realsense_ros2_camera)

# ROS2 Flags
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

find_package(realsense2)
if(NOT realsense2_FOUND)
  message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

if(CMAKE_BUILD_TYPE EQUAL "RELEASE")
  message(STATUS "Create Release Build.")
  set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
  message(STATUS "Create Debug Build.")
endif()

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(realsense_camera_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

add_library(${PROJECT_NAME}-lib
  include/${PROJECT_NAME}/realsense_camera_node.hpp
  include/${PROJECT_NAME}/constants.hpp
)

# Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(RealSenseROS2::${PROJECT_NAME}-lib ALIAS ${PROJECT_NAME}-lib)

target_compile_definitions(${PROJECT_NAME}-lib PRIVATE "AMENT_INDEX_CPP_BUILDING_DLL")

# Set target properties
target_include_directories(${PROJECT_NAME}-lib
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  INTERFACE
    ${rclcpp_INCLUDE_DIRS}
    ${builtin_interfaces_INCLUDE_DIRS}
    ${rmw_implementation_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${realsense_camera_msgs_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}-lib
  PRIVATE
    ${rclcpp_LIBRARIES}
    ${rmw_implementation_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_LIBRARIES}
    ${realsense_camera_msgs_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ${realsense2_LIBRARY}
    ${image_transport_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}-lib PROPERTIES LINKER_LANGUAGE CXX)
target_compile_features(${PROJECT_NAME}-lib PRIVATE cxx_auto_type)

ament_export_dependencies(${PROJECT_NAME}-lib
  rclcpp
  rmw_implementation
  std_msgs
  sensor_msgs
  tf2_ros
  tf2
  realsense_camera_msgs
  cv_bridge
  OpenCV
  image_transport
  librealsense2
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "AMENT_INDEX_CPP_BUILDING_DLL")

# Set target properties
target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${PROJECT_NAME}-lib
)

target_include_directories(${PROJECT_NAME} PUBLIC
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
	"$<INSTALL_INTERFACE:include>"
)

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/RealSenseROS2)

# Install library
install(TARGETS ${PROJECT_NAME}-lib
  EXPORT ${PROJECT_NAME}-lib-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

#This is required so that the exported target has the name RealSenseROS2
set_target_properties(${PROJECT_NAME}-lib PROPERTIES EXPORT_NAME RealSenseROS2)

# Install binaries
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

#Export the targets to a script
install(EXPORT ${PROJECT_NAME}-lib-targets
  FILE
  RealSenseROS2Targets.cmake
  NAMESPACE
    RealSenseROS2::
  DESTINATION
    ${INSTALL_CONFIGDIR}
)

#Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/RealSenseROS2ConfigVersion.cmake
    VERSION 1.0
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/RealSenseROS2Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/RealSenseROS2Config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

#Install the config, configversion and custom find modules
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/RealSenseROS2Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/RealSenseROS2ConfigVersion.cmake
    #DESTINATION ${INSTALL_CONFIGDIR}
    DESTINATION share/${PROJECT_NAME}/
)

# Exporting from the build tree
export(EXPORT ${PROJECT_NAME}-lib-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/RealSenseROS2Targets.cmake NAMESPACE RealSenseROS2::)

#Register package in user's package registry
export(PACKAGE ${PROJECT_NAME}-lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(REALSENSE_DEVICE_PLUGIN FALSE)
  if(${REALSENSE_DEVICE_PLUGIN})
    ament_add_gtest(test_api test/test_api.cpp)
  endif()
  if(TARGET test_api)
    target_include_directories(test_api PUBLIC
      ${${PROJECT_NAME}_INCLUDE_DIRS}
    )
    ament_target_dependencies(test_api
      rclcpp
      sensor_msgs
      tf2_ros
      tf2
      OpenCV)
  endif()
endif()

ament_package()
