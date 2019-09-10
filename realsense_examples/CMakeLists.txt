cmake_minimum_required(VERSION 3.5)
project(realsense_examples)

find_package(ament_cmake REQUIRED)
find_package(realsense_ros REQUIRED)
find_package(realsense_node REQUIRED)


install(DIRECTORY 
        launch 
        config 
        DESTINATION share/${PROJECT_NAME})

ament_package()
