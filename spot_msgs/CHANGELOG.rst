^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spot_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2023-08-06)
------------------

1.0.0 (2023-04-07)
------------------
* update maintainer
* add actionserver for docking/undocking
* updates after testing picking and placing on real spot - added some useful arguments to msg definitions
* updates from testing on real robot
* implemented grasp service
* changed to fix sized arrays
* changed hand pose msg
* service for getting dock state
* Create HandPose.srv
* Update CMakeLists.txt
* New msg for arm force trajectory
* Update CMakeLists.txt
* add posed_stand service which can be used to pose the body
  This also adds a check to the async_idle which prevents the idle stand command
  from activating while there is a command which has a non-None id. This should
  prevent commands from getting interrupted by that stand
* add terrain state to foot state, publish tf of foot position rt body
* Advanced motion tab, obstacle and terrain params added to driver and panel
  Move more advanced motion settings into the adv. motion tab to not clutter the basic motion. The settings there (ground friction, gait, swing height, grated surface mode) are I suspect relatively rarely used.
  Obstacle params message added, which allows the settings for the obstacle avoidance to be published from the driver. These are as usual empty by default since the SDK does not provide details. Currently the only modification allowed is to the obstacle padding.
  Terrain params message added, which allows setting of the ground friction and grated surfaces mode.
  Terrain and obstacle params are in the mobility params message published by the driver.
  Rviz panel allows interaction with the terrain and obstacle params. Comboboxes are now initialised by using a map generated from the constants in the ros messages, to try and make sure the ordering is correct. The map is also useful for constants which are non-consecutive like in the locomotion, for some reason the 9th value is unused but the 10th is.
* velocity and body controls are in different tabs, add swing height and gait control
  add checks to ensure only valid values for locomotion and swing height are sent to the mobility params
* allow velocity limits to be set in the driver launch and load them on start
  Also updats the mobility params message to store the velocity limits
  If the limits change, the rviz panel is updated with the latest values
* Service to open the gripper at a given angle
* Added messages for the arm
* add Dock.srv
* add spot dock and undock service
* use int8 instead of uint8, we want to have negative rotations too
* add actionserver to set body pose
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
* [spot_msgs] add descriptions to SetLocomotion.srv
* [spot_msgs] remove unused field of mobility params message
* [spot_msgs] add MobilityParams.msg
* [spot_msgs] fix type error of SetLocomotion.srv
* [spot_msgs] add SetLocomotion.srv
* support graphnav navigate-to
* Properly implemented checking the status of commands.  Added a feedback message
* Fixed all the little mistakes from the last commit
* Added messages for the remainder of the robot status
* Added messages and brought metrics and leases into ros
* Contributors: Chris Iverach-Brereton, Dave Niewinski, Esther, Kei Okada, Koki Shinjo, Michal Staniaszek, Michel Heinemann, Naoto Tsukamoto, SpotCOREAI, maubrunn, nfilliol
