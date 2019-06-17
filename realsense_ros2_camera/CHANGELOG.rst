^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_ros2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2018-12-11)
------------------
* Merge pull request `#18 <https://github.com/intel/ros2_intel_realsense/issues/18>`_ from nuclearsandwich/add-dependencies-to-package.xml
  Add eigen as a build dependency for ros2debian
* Add eigen as a build dependency.
* Contributors: Chris Ye, Steven! Ragnarök

2.0.4 (2019-05-30)
------------------
* Merge pull request `#50 <https://github.com/intel/ros2_intel_realsense/issues/50>`_ from ahuizxc/dashing
  fix compiling failure after ros2 core updated and remove some warnings.
* fix compiling failure after ros2 core updated and remove some warnings.
* Merge pull request `#44 <https://github.com/intel/ros2_intel_realsense/issues/44>`_ from ahuizxc/master
  add depthimage to laser scan launch file
* add depthimage to laser scan
* enable librealsensev2.17.1
  enable librealsensev2.17.1
* update package.xml and readme
* enable librealsensev2.17.1
* Merge pull request `#39 <https://github.com/intel/ros2_intel_realsense/issues/39>`_ from challen-zhou/master
  Add options to enable/disable color-aligned point cloud
* Change aligned pointcloud format from XYZ to XYZRGB
* Add options to enable/disable color-aligned point cloud
  Since the default 2 point cloud process cost too much computation,
  add option to enable/disable(default) color-aligned point cloud.
* Merge pull request `#36 <https://github.com/intel/ros2_intel_realsense/issues/36>`_ from challen-zhou/master
  Add pointcloud aligned with color
* Add pointcloud aligned with color
  Add pointcloud generate from depth which aligned to color,
  no RGB channel since this pointcloud one-to-one matches with color image.
* Merge pull request `#32 <https://github.com/intel/ros2_intel_realsense/issues/32>`_ from challen-zhou/master
  Update depth alignment function to align with librealsense
* Merge pull request `#30 <https://github.com/intel/ros2_intel_realsense/issues/30>`_ from ahuizxc/master
  pass the format test and fix ctrl+c error
* Update depth alignment function to align with librealsense
  Update depth alignment function to align with librealsense since
  librealsense already upstreamed to ROS2/Crystal, also we can benifit from
  the future librealsense optimization.
* fix ctrl+c error
* pass the format test
* Merge pull request `#29 <https://github.com/intel/ros2_intel_realsense/issues/29>`_ from ahuizxc/master
  fixed bug that camera not work when enable_depth=False
* fixed bug that camera not work when enable_depth=False
* Merge pull request `#23 <https://github.com/intel/ros2_intel_realsense/issues/23>`_ from intel/fix_ctest
  fix error when run CTest
* fix error when run CTest
  * set Realsense device not exist by default, disable testapi when Realsense device not plugin
  * fix format error when run CTest
* Merge pull request `#22 <https://github.com/intel/ros2_intel_realsense/issues/22>`_ from RachelRen05/update_readme
  update project to install debain package dependency
* update readme.md
  * update project to install debain package dependency
  * add rviz default configuration and support ros2 launch
* update maintainer
* 2.0.3
* update changelog
* Merge pull request `#18 <https://github.com/intel/ros2_intel_realsense/issues/18>`_ from nuclearsandwich/add-dependencies-to-package.xml
  Add eigen as a build dependency for ros2debian
* Add eigen as a build dependency.
* Contributors: Chris Ye, Gary Liu, RachelRen05, Sharron LIU, Steven! Ragnarök, Xiaojun Huang, ahuizxc, challenzhou

2.0.2 (2018-12-07)
------------------
* Merge pull request `#15 <https://github.com/intel/ros2_intel_realsense/issues/15>`_ from intel/pointer_api
  create a Subscriber using a raw pointer
* create a Subscriber using a raw pointer
  std::shared_ptr is not safe, replace with new pointer API from image_transport (https://github.com/ros-perception/image_common/pull/104)
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#16 <https://github.com/intel/ros2_intel_realsense/issues/16>`_ from intel/enable_realsense
  update librealsense dependence
* update Readme to modify librealsense guide
* added librealsense2 dependence
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* create wrapper class PipelineSyncer to work around librealsense 2.16 feature, removing operator() from class asynchronous_syncer
  merge from https://github.com/IntelRealSense/librealsense/commit/985f3b75539ec08a78db69c9f8f2505d3d0c2e40
  Fix no match for call to '(rs2::asynchronous_syncer) (rs2::frame)' issue: https://github.com/IntelRealSense/librealsense/issues/2314
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#13 <https://github.com/intel/ros2_intel_realsense/issues/13>`_ from yechun1/image_transport
  publish raw image by image_transport publisher
* fix typo and add transport in Readme
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* publish raw image by image_transport publisher
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#14 <https://github.com/intel/ros2_intel_realsense/issues/14>`_ from yechun1/fix_build_issue
  add semicolon after RCLCPP_WARN to fix build issue
* add semicolon after RCLCPP_WARN to fix build issue
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Merge pull request `#12 <https://github.com/intel/ros2_intel_realsense/issues/12>`_ from sharronliu/master
  camera node: replaced RCUTILS with RCLCPP logger
* camera node: replaced RCUTILS with RCLCPP logger
  Signed-off-by: Sharron LIU <sharron.liu@intel.com>
* Merge pull request `#11 <https://github.com/intel/ros2_intel_realsense/issues/11>`_ from ahuizxc/aligned_depth
  enable aligned depth image
* corrected the xml version
* keep the topics' name same as ros1, add option _align_depth to enable or disable aligned_depth_to_color
* fix frame_id of aligned depth image
* fixed format error
* enable aligned depth image
* Contributors: Chris Ye, Sharron LIU, ahuizxc
