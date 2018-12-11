^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_ros2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2018-12-11)
------------------
* Merge pull request `#18 <https://github.com/intel/ros2_intel_realsense/issues/18>`_ from nuclearsandwich/add-dependencies-to-package.xml
  Add eigen as a build dependency for ros2debian
* Add eigen as a build dependency.
* Contributors: Chris Ye, Steven! Ragnarök

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
