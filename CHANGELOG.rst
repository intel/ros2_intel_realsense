^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for ros2_intel_realsense
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
2.0.6 (2019-09-03)
------------------
* support multi camera
* support hot plugin and automatic reconnection

2.0.5 (2019-08-30)
------------------
* Support T265 and D435i
* Support reconfigure parameters at runtime
* Support the latest release(v2.26.0) of librealsense
* Implement ROS2 realsense node as a component which can be loaded at runtime and adopt the advanced features in ROS2 dashing release, e.g. intra-process communication.

2.0.3 (2018-12-11)
------------------
* fix build dependence for ros2 crystal release
* Add eigen as a build dependency.

2.0.2 (2018-12-07)
------------------
* create for bloom release


2.0.1 (2018-05-31)
--------------------------------------------------------------------------------
Sharron LIU (12):
      ros2_intel_realsense: added initial version
      README: updated for repo name "ros2_intel_realsense"
      Merge pull request #1 from dongx1x/master
      camera_msgs: added test dependent ament_lint_common
      realsense_ros2_camera: added test dependency ament_lint_common
      realsense_ros2_camera: copyright notes in ROS2 style
      ament test: pass ament test
      Merge pull request #3 from sharronliu/master
      cv_bridge: use the upstream ros2 cv_bridge
      Merge pull request #4 from sharronliu/master
      realsense_ros2_camera: added test dependent ros package
      README: clarified host OS supported

Xiaocheng Dong (1):
      Add tests

dongx1x (1):
      Update README.md

