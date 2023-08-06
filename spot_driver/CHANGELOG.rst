^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spot_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2023-08-06)
------------------

1.0.0 (2023-04-07)
------------------
* add python-transforms3d-pip to package xml
* add odometry topic which outputs twist in correct frame, fixes `#95 <https://github.com/heuristicus/spot_ros/issues/95>`_
* subscriber check now polls also depth/info topics
* fix tf repeated message spam when publishing tfs at high frequency
* increase velocity command timeout to 0.6 seconds
  probably fixes `#96 <https://github.com/heuristicus/spot_ros/issues/96>`_
* complete revision of the documentation mostly up to date with the current driver state
* fix incorrect actionserver in use for in motion body pose
* update maintainer
* robots without arms do not require manipulation client
* black formatting
* connect up buttons, fix undock call, fix dock actionserver name
* fix last docking command assignment, reset last stand command on dock. Fixes `#110 <https://github.com/heuristicus/spot_ros/issues/110>`_
* add actionserver for docking/undocking
* Fix spam when point cloud client is not initialised because there is no EAP
* changed rospy to logging.Logger dependency
* removed arm controller trial
* changed to correct tasks list
* changed list name
* changed lookup name
* changed lookup definiton
* only add tasks when needed
* updates after testing picking and placing on real spot - added some useful arguments to msg definitions
* starting to implement controller action server for moveit support
* implemented grasp service
* apply black formatting and add black check on PRs and pushes
* only add hand camera task if arm
* corrected index
* added warn messages for out of range
* reviewed limits for spot arm
* changed gripper_open_angle to take degrees
* adressed layout review comments
* changed to fix sized arrays
* changed hand pose msg
* fixed indentation and removed walk to pixel method
* add RollOver functions
* include fix for failing service call
* driver start process loops where it can fail due to the robot still booting. More messages about what it is doing.
* posed stand actionserver sleeps a bit before returning to allow the motion to complete
* implemented helper function for DockState message
* use correct action and service fields, set actionserver result
* add an actionserver for stand commands
* Driver config file can be passed as an arg to driver launch
* ROS loop rate can be set in params to allow higher publication frequencies
* Wrapper method to walk to object
* Subscriber to walk to object in picture
  ROS_NAMESPACE=spot rosrun my_grasping image_box.py
  Then click on pixel to walk to
* service for getting dock state
* print error messages when the estop keepalive has errors, initialise keepalive to none in init
* autonomy is not allowed if the robot is on shore power, cmd_vel counts as autonomy
* Created service for gripper pose
* Added method to send location to gripper
* rename old body_pose topic to try and reflect what it actually does
* Methods: body to follow arm, arm force trajectory
* Services for body follow arm, force trajectory arm
* add posed_stand service which can be used to pose the body
  This also adds a check to the async_idle which prevents the idle stand command
  from activating while there is a command which has a non-None id. This should
  prevent commands from getting interrupted by that stand
* Updated the way to ensure power on & spot standing
* add terrain state to foot state, publish tf of foot position rt body
* Method added to check Spot is powered on and standing
  New method: check_arm_power_and_stand
* Removed Estop check before arm moves
* Change method name
  Changed method's name from "make_robot_command" to "make_arm_trajectory_command" for clarity
* Removed unnecessary code for testing
  Powering off robot after moving the arm was removed; removed unnecessay sleep when moving the arm
* Added carry option for arm and hand images
  Hand images from EricVoll
* Added hand image rate
* tab shown on startup should be basic motion tab, minor fixes
* allowed to move check has arg which specifies if autonomy enabled should be checked
* sit, stand and self right disabled when motion disabled, ui has button for allow motion
  The stop button in the ui will now call the locked stop. Stop button is disabled and has its text changed when motion is not allowed
* add locked_stop service which interrupts current motion and disallows further motion
  Publisher for the state of allow motion
* Advanced motion tab, obstacle and terrain params added to driver and panel
  Move more advanced motion settings into the adv. motion tab to not clutter the basic motion. The settings there (ground friction, gait, swing height, grated surface mode) are I suspect relatively rarely used.
  Obstacle params message added, which allows the settings for the obstacle avoidance to be published from the driver. These are as usual empty by default since the SDK does not provide details. Currently the only modification allowed is to the obstacle padding.
  Terrain params message added, which allows setting of the ground friction and grated surfaces mode.
  Terrain and obstacle params are in the mobility params message published by the driver.
  Rviz panel allows interaction with the terrain and obstacle params. Comboboxes are now initialised by using a map generated from the constants in the ros messages, to try and make sure the ordering is correct. The map is also useful for constants which are non-consecutive like in the locomotion, for some reason the 9th value is unused but the 10th is.
* velocity and body controls are in different tabs, add swing height and gait control
  add checks to ensure only valid values for locomotion and swing height are sent to the mobility params
* additional layer of control over whether motion is allowed, which can be changed while the driver is running
* reject velocity limits in the range 0 < lim < 0.15 as there can be issues with the trajectory command
* allow velocity limits to be set in the driver launch and load them on start
  Also updats the mobility params message to store the velocity limits
  If the limits change, the rviz panel is updated with the latest values
* initialise stop service after actionservers that it calls to prevent crashes if called during startup
* add a flag to explicitly enable autonomous functions
* add ros param to set the estop timeout, wrapper takes it as an arg
* track when the trajectory command returns status unknown, and try to resend the command once
* tolist() -> tobytes()
* not build client if no point_cloud services are available
* fix data type float32 -> uint8
* add lidar topic publisher, tf broadcaster
* ros helpers with PointCloud
* publish pointcloud from VLP16
* Service for gripper open at an angle
* Wrapper updated to open gripper at a given angle
* Implemented Close/open gripper services
* Added open & close gripper services
* Added stow, unstow, and control joints of the arm commands.
* add spot dock and undock service
* add ros param to set the estop timeout, wrapper takes it as an arg
* calling the stop service now preempts actionservers if they are active
* no longer reject trajectory poses which are not in body frame, just transform them
* add topic go_to_pose which can be used to move the robot with the trajectory command
* rename max_velocity to velocity_limit, now also limits velocity when moving backwards
* add actionserver to set body pose
* Add an additional envar to disable auto-launching the Velodyne as this _could\_ cause network issues
* Fix a mismatched tag
* Add the teleop_joy dependency
* Use the velodyne_description package for the actual sensor mesh, use the cage as a separate entity. Add an accessories.launch file to automatically bring up the velodyne if needed
* Add the bluetooth_teleop node, default config file.
* Simple rviz panel for interaction with spot
  Can use the panel to claim and release the lease, power on and off, sit down and stand up, and set the body pose.
  To add, go to panels>add new panel and select SpotControlPanel in spot_viz
* can require trajectory commands to reach goal precisely
  Add precise_positioning field to the trajectory goal. Setting to true will make the wrapper consider only STATUS_AT_GOAL status to mean that the robot is at the goal. Setting to false will replicate previous behaviour where STATUS_NEAR_GOAL also means that the robot is at the goal.
  Vary feedback messages from the driver depending on whether the precise positioning field was set.
* Include the license file in the individual ROS packages
* Add service call to set maximum velocity, use synchro trajectory command
  The service call receives a twist message and sets the mobility params vel_lim max_vel to the linear x and y, and angular z values in the message. This velocity limit affects anything that moves the robot around, such as the trajectory command and velocity command.
  The service call adds a srv to spot_msgs
  Output exception string when power_on command fails
  Fix minor typo in behavior_fault function name
  Populate state_description in the estop state message
  Minor cosmetic changes to srv imports and some comments for readability
* Updated E-Stop to use keepalive client rather than the endpoint. Fixes a bug where the E-Stop would release immeaditly after being triggered (`#38 <https://github.com/heuristicus/spot_ros/issues/38>`_)
  Co-authored-by: marble-spot <D01@marble.com>
* Add trajectory command interface (`#25 <https://github.com/heuristicus/spot_ros/issues/25>`_)
  * [spot_driver] add trajectory_cmd() method to spot_wrapper
  * [spot_driver] rename _last_motion_command to _last_trajectory_command
  * [spot_driver] rename _last_motion_command_time to _last_velocity_command_time
  * [spot_driver] fix trajectory_cmd
  * [spot_driver] fix options of trajectory_cmd()
  * [spot_msgs] add Trajectory.srv
  * [spot_driver] add trajectory service server
  * [spot_ros] add frame_id checking to trajectory_cmd
  * [spot_driver] fix bugs
  * [spot_driver] fix bugs in trajectory_cmd
  * convert trajectory command to an actionserver
  * [spot_ros] fix merge commit
  * [spot_driver] allow STATUS_NEAR_GOAL to be recognized as at_goal
  * [spot_driver] add handling for 0 duration
  Co-authored-by: Michal Staniaszek <m.staniaszek@gmail.com>
  Co-authored-by: Dave Niewinski <dniewinski@clearpathrobotics.com>
* Added service for clearing behavior faults (`#34 <https://github.com/heuristicus/spot_ros/issues/34>`_)
* Create runtime static transform broadcaster for camera transforms (`#31 <https://github.com/heuristicus/spot_ros/issues/31>`_)
  The transforms between the body and camera frames are static and will not change. This change checks the transform for static frames once when an image with the required data is received. It then stores the transform to be published by a static transform publisher.
  Previously when the robot was moved quickly there would be a lag between the camera frames updating and the actual position of the robot. With static transforms the cameras are always in the correct position relative to the body.
* Package structure more in line with ROS recommendation for python, works on noetic (`#32 <https://github.com/heuristicus/spot_ros/issues/32>`_)
  * install graph_nav_util with catkin_install_python
  * fix usage of Image.Format to be consistent with other usages
  Using Image.Format.FORMAT_RAW causes an AttributeError:
  AttributeError: 'EnumTypeWrapper' object has no attribute 'FORMAT_RAW'
  The usage here was inconsistent with usages in other files.
  * fix python package structure to work on noetic
  * fix shebang line in spot_ros, explicitly specify use of python3
* Updated command feedback protos and expanded twist mux
* Fix image format and include graph_nav_util in install (`#29 <https://github.com/heuristicus/spot_ros/issues/29>`_)
  * install graph_nav_util with catkin_install_python
  * fix usage of Image.Format to be consistent with other usages
  Using Image.Format.FORMAT_RAW causes an AttributeError:
  AttributeError: 'EnumTypeWrapper' object has no attribute 'FORMAT_RAW'
  The usage here was inconsistent with usages in other files.
* changed to use ros logging instead of print (`#14 <https://github.com/heuristicus/spot_ros/issues/14>`_)
  * [spot_driver] change print() to ros logging
  * [spot_driver] use ros logger instead of print()
* Updated some deprecated functions
* [spot_driver] bugfix for walking-mode
* [spot_driver] add mobility params publisher
* [spot_driver] fix locomotion_mode service
* [spot_driver] add locomotion_mode service
* [spot_driver] add stair_mode service to spot_ros
* [spot_driver] add get_mobility_params method and change set_mobility_params method
* fix for version v2.2.1, skip waypint\_ instead of waypoint
* support graphnav navigate-to
* add missing assignment to w field of quaternion
* Changed cmd_vel queue size to 1 from infinite to prevent controller lag
* Minor workaround for invalid timestamps
* [spot_driver] changed the names of odometry frames to default
* [spot_driver] changed default odom to kinematic odometry
* [spot_driver] add exception handlings to AsyncIdle class
* Publish /odom
* [spot_driver] add rosparams to determine a parent odometry frame
* [spot_driver] delete commented lines and redundant variables
* Publish /vision tf as the parent of /body tf
* [spot_driver] add python3-rospkg-modules dependency
* Use 16UC1 instead of mono16 for depth image encoding
* add twist_mux, interactive_marker_twist_server dependency
* Removed token as of api 2.0
* Moved resetting and claiming estop out of spot ros startup
* add install instructions
* Updated URDF to include mount points, and deps for install
* Incorporating clock skew between spot and ROS machine
* Initial stab as accomodating skew between systems.  Untested
* Properly implemented checking the status of commands.  Added a feedback message
* Made the driver not automatically claim a body lease and e stop.  Allows you to monitor without having control.  Disconnect doesn't exit very gracefully.  Need to wait on sitting success
* Not defining a rate or defining a rate of <= 0 will disable a data source
* Updated image type selection to use protobuf enums
* Disabled auto standing by default.  Updated api version numbering
* Updated dependencies
* Fixed mobility params when switching between movement and stationary
* Lots of little fixes.  Body positioning logic is broken
* Error-checking improved
* removed unnecessary interactive marker
* Updated the estop setup so it is externally controllable
* Mobility parameters implemented.  Some stuff still hard-coded
* Basic motion.  Lots to clean up
* Added lots of docstrings
* Added services for power, stand, sit, self-right, and stop
* Updated doc outline.  Still missing updated docstrings
* Refactoring to keep all the ros message building separate
* Got depth images working
* Added rosdoc
* Added image streams into ROS
* Fixed all the little mistakes from the last commit
* Blocked-out remainder of robot state callback.  Untested
* Added messages and brought metrics and leases into ros
* Refactored the ros driver to be cleaner.  Added a lot of docstrings.  Incomplete
* Squashed commit of the following:
  commit 0424b961e75b0e8f4143424e9fb0121ee5b3c01c
  Author: Dave Niewinski <dniewinski@clearpathrobotics.com>
  Date:   Mon May 11 16:36:55 2020 -0400
  Updated logging
  commit afdc5301f2b73f219b51ae3ce7c56e0f036e75a6
  Author: Dave Niewinski <dniewinski@clearpathrobotics.com>
  Date:   Mon May 11 15:00:27 2020 -0400
  Added launch and params
  commit 8c1066108d3cc2955cf49a73a75e3d249a8704d2
  Author: Dave Niewinski <dniewinski@clearpathrobotics.com>
  Date:   Fri May 8 15:04:48 2020 -0400
  Basic ros support implemented.  Outputs joint angles, and odom
  commit 3f71252b182738234cc54e581cac3b8a54874733
  Author: Dave Niewinski <dniewinski@clearpathrobotics.com>
  Date:   Wed May 6 16:46:21 2020 -0400
  Basic functionality, just printing in the terminal.
  commit 505c17e1d4d5a28d14872d81e2f11b60b61135e9
  Author: Dave Niewinski <dniewinski@clearpathrobotics.com>
  Date:   Wed May 6 14:00:42 2020 -0400
  Initial pass at data from robot
* Contributors: Chris Iverach-Brereton, Dave Niewinski, Esther, Harel Biggie, Kei Okada, Koki Shinjo, Maurice Brunner, Michal Staniaszek, Michel Heinemann, Naoto Tsukamoto, Naoya Yamaguchi, SpotCOREAI, Telios, Wolf Vollprecht, Yoshiki Obinata, harelb, jeremysee2, maubrunn, nfilliol
