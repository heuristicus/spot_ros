Configuring the ROS Driver
==========================

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
| Parameter              | Description                                                                     |
+========================+=================================================================================+
| rates/robot_state      | The robot's state (include joint angles)               |
+------------------------+---------------------------------------------------------------------------------+
| rates/metrics          | The robot's metrics                                    |
+------------------------+---------------------------------------------------------------------------------+
| rates/lease            | The robot's lease state                                |
+------------------------+---------------------------------------------------------------------------------+
| rates/front_image      | The image and depth image from the front camera        |
+------------------------+---------------------------------------------------------------------------------+
| rates/side_image       | The image and depth image from the side cameras        |
+------------------------+---------------------------------------------------------------------------------+
| rates/loop_frequency   | The image and depth image from the rear camera         |
+------------------------+---------------------------------------------------------------------------------+
| rates/point_cloud      | The point cloud from the EAP lidar                     |
+------------------------+---------------------------------------------------------------------------------+
| rates/hand_image       | The image and depth image from the arm's hand camera   |
+------------------------+---------------------------------------------------------------------------------+
| rates/feedback         | A feedback message with various information           |
+------------------------+---------------------------------------------------------------------------------+
| rates/mobility_params  | The image and depth image from the rear camera         |
+------------------------+---------------------------------------------------------------------------------+
| rates/check_subscribers| Check for subscribers on camera topics. Data is not published without subscribers   |
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