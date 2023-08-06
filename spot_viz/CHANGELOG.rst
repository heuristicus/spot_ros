^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spot_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2023-08-06)
------------------

1.0.0 (2023-04-07)
------------------
* docking, self-right and roll over buttons start disabled
* update maintainer
* connect up buttons, fix undock call, fix dock actionserver name
* add buttons for docking, self-right, and roll over
* body tab now sets posed stand
* update rviz config panel arrangement, estop states now shown in panel
  The release estop button is now only enabled if the software estop is on
* correct ordering of ui component setup, must be before subscribers
* sit and stand button state depends on whether the robot is powered or not
* tab shown on startup should be basic motion tab, minor fixes
* sit, stand and self right disabled when motion disabled, ui has button for allow motion
  The stop button in the ui will now call the locked stop. Stop button is disabled and has its text changed when motion is not allowed
* Advanced motion tab, obstacle and terrain params added to driver and panel
  Move more advanced motion settings into the adv. motion tab to not clutter the basic motion. The settings there (ground friction, gait, swing height, grated surface mode) are I suspect relatively rarely used.
  Obstacle params message added, which allows the settings for the obstacle avoidance to be published from the driver. These are as usual empty by default since the SDK does not provide details. Currently the only modification allowed is to the obstacle padding.
  Terrain params message added, which allows setting of the ground friction and grated surfaces mode.
  Terrain and obstacle params are in the mobility params message published by the driver.
  Rviz panel allows interaction with the terrain and obstacle params. Comboboxes are now initialised by using a map generated from the constants in the ros messages, to try and make sure the ordering is correct. The map is also useful for constants which are non-consecutive like in the locomotion, for some reason the 9th value is unused but the 10th is.
* velocity and body controls are in different tabs, add swing height and gait control
  add checks to ensure only valid values for locomotion and swing height are sent to the mobility params
* panel now looks at battery and power state and displays information
* allow velocity limits to be set in the driver launch and load them on start
  Also updats the mobility params message to store the velocity limits
  If the limits change, the rviz panel is updated with the latest values
* enable estop buttons only when we have the lease, release button state is dependent on estop state
* add stop action button to rviz panel, clearer estop button names
* add estop buttons to spot panel
* add topic go_to_pose which can be used to move the robot with the trajectory command
* adjust velocity limit and body height range, explicitly specify upper and lower limits for labels
  Trying to adjust body height, the limit appears to be something around 0.15 or so. I think this might be because we are not using the stance command, but the body control params for mobility
* rename max_velocity to velocity_limit, now also limits velocity when moving backwards
* Only update the state of the buttons based on the lease feedback; if we successfully get/release a lease, disable that button only and wait for the subscriber to enable whatever's needed.  This prevents a possible issue where the UI flashes
* Squashed commit of the following:
  commit 0cc926539660598bf7ef2a8407d51a3583ca29c9
  Author: Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
  Date:   Fri Aug 13 11:49:21 2021 -0400
  Change the default camera to use gpe as the fixed frame; the keeps the robot visible on-screen at all times; using odom the robot can appear off-screen too easily
  commit 667e26f63ad9df414c4bd7f02eea57d03e2785d3
  Author: Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
  Date:   Fri Aug 13 11:45:16 2021 -0400
  Add a subscriber to the spot panel to monitor the lease status, correctly setting the button states if we call the service e.g. before launching rviz
* Simple rviz panel for interaction with spot
  Can use the panel to claim and release the lease, power on and off, sit down and stand up, and set the body pose.
  To add, go to panels>add new panel and select SpotControlPanel in spot_viz
* can set maximum velocity in the rviz panel
* Simple rviz panel for interaction with spot
  Can use the panel to claim and release the lease, power on and off, sit down and stand up, and set the body pose.
  To add, go to panels>add new panel and select SpotControlPanel in spot_viz
* Include the license file in the individual ROS packages
* add install instructions
* Updated PC topics
* Lots of little fixes.  Body positioning logic is broken
* removed unnecessary interactive marker
* Basic motion.  Lots to clean up
* Got depth images working
* Initial commit
* Contributors: Chris Iverach-Brereton, Dave Niewinski, Michal Staniaszek, Wolf Vollprecht
