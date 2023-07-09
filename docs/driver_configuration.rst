Driver configuration
====================

There are some basic settings that can be configured to control how the driver runs and at what rate it obtains data.
Data that is more important for your applications can be obtained at a higher rate.


The default configuration file is located at ``spot_driver/config/spot_ros.yaml``

Rate parameters
---------------
These parameters control the rate of various callbacks and loops in the driver.

.. note::

  These rates are requests and not guarantees. The rate may vary depending on your network configuration and other
  factors.

+------------------------+---------------------------------------------------------------------------------+
|Parameter               |Description                                                                      |
+========================+=================================================================================+
|rates/robot_state       |The robot's state (include joint angles)                                         |
+------------------------+---------------------------------------------------------------------------------+
|rates/metrics           |The robot's metrics                                                              |
+------------------------+---------------------------------------------------------------------------------+
|rates/lease             |The robot's lease state                                                          |
+------------------------+---------------------------------------------------------------------------------+
|rates/front_image       |The image and depth image from the front camera                                  |
+------------------------+---------------------------------------------------------------------------------+
|rates/side_image        |The image and depth image from the side cameras                                  |
+------------------------+---------------------------------------------------------------------------------+
|rates/loop_frequency    |The image and depth image from the rear camera                                   |
+------------------------+---------------------------------------------------------------------------------+
|rates/point_cloud       |The point cloud from the EAP lidar                                               |
+------------------------+---------------------------------------------------------------------------------+
|rates/hand_image        |The image and depth image from the arm's hand camera                             |
+------------------------+---------------------------------------------------------------------------------+
|rates/feedback          |A feedback message with various information                                      |
+------------------------+---------------------------------------------------------------------------------+
|rates/mobility_params   |Parameters indicating the robot's mobility state                                 |
+------------------------+---------------------------------------------------------------------------------+
|rates/check_subscribers |Check for subscribers on camera topics. Data is not published without subscribers|
+------------------------+---------------------------------------------------------------------------------+

Startup parameters
-------------------

There are several parameters which can be used to automatically do certain actions when the driver starts.

.. warning::

  Having the robot power its motors and stand after the driver connects is a physical action is inherently dangerous.
  Consider whether you need these settings, and ensure the area is clear before starting the driver.

+------------------------+---------------------------------------------------------------------------------+
| Parameter              | Description                                                                     |
+------------------------+---------------------------------------------------------------------------------+
| auto_claim             | A boolean to automatically claim the body and e stop when the driver connects   |
+------------------------+---------------------------------------------------------------------------------+
| auto_power_on          | A boolean to automatically power on the robot's motors when the driver connects |
+------------------------+---------------------------------------------------------------------------------+
| auto_stand             | A boolean to automatically stand the robot after the driver conneccts           |
+------------------------+---------------------------------------------------------------------------------+

By default, the driver publishes the odom and gpe transformation with its origin at the start-up location. 
When using a Global Positioning System like GNSS, this can be disabled to use an external position estimator.

+------------------------+---------------------------------------------------------------------------------+
| Parameter              | Description                                                                     |
+------------------------+---------------------------------------------------------------------------------+
| publish_odom_tf        | A boolean to determine if the driver should publish the odom and gpe tf         |
+------------------------+---------------------------------------------------------------------------------+


Environment variables
---------------------

Some environment variables can modify the behaviour or configuration of the driver.

+----------------------------+-----------------------------------+----------------------------------------------------------------------------+
|Variable                    |Default Value                      |Description                                                                 |
+============================+===================================+============================================================================+
|SPOT_VELODYNE_HOST          |192.168.131.20                     |IP address of the VLP-16 sensor                                             |
+----------------------------+-----------------------------------+----------------------------------------------------------------------------+
|SPOT_JOY_DEVICE             |/dev/input/js0                     |The Linux joypad input device used by the joy_teleop node                   |
|                            |                                   |                                                                            |
+----------------------------+-----------------------------------+----------------------------------------------------------------------------+
|SPOT_JOY_CONFIG             |spot_control/config/teleop.yaml    |Joypad button/axis configuration file for joy_teleop                        |
|                            |                                   |                                                                            |
+----------------------------+-----------------------------------+----------------------------------------------------------------------------+
|SPOT_VELODYNE_AUTOLAUNCH    |1                                  |If 1 and SPOT_VELODYNE is also 1, the VLP16 ROS node will start             |
|                            |                                   |automatically                                                               |
+----------------------------+-----------------------------------+----------------------------------------------------------------------------+