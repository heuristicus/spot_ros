Configuring the ROS Driver
==========================

There are some basic settings that can be configured to control how the driver runs and at what rate it obtains data.  Data that is more important for your applicaiton can be obtained at a higher rate.

The default configuration file is located at ``spot_driver/config/spot_ros.yaml``

+------------------------+---------------------------------------------------------------------------------+
| Parameter              | Description                                                                     |
+========================+=================================================================================+
| rates/robot_state      | The rate in Hz to obtain the robot's state (include joint angles)               |
+------------------------+---------------------------------------------------------------------------------+
| rates/metrics          | The rate in Hz to obtain the robot's metrics                                    |
+------------------------+---------------------------------------------------------------------------------+
| rates/lease            | The rate in Hz to obtain the robot's lease state                                |
+------------------------+---------------------------------------------------------------------------------+
| rates/front_image      | The rate in Hz to obtain the image and depth image from the front camera        |
+------------------------+---------------------------------------------------------------------------------+
| rates/side_image       | The rate in Hz to obtain the image and depth image from the side cameras        |
+------------------------+---------------------------------------------------------------------------------+
| rates/rear_image       | The rate in Hz to obtain the image and depth image from the rear camera         |
+------------------------+---------------------------------------------------------------------------------+
| auto_claim             | A boolean to automatically claim the body and e stop when the driver connects   |
+------------------------+---------------------------------------------------------------------------------+
| auto_power_on          | A boolean to automatically power on the robot's motors when the driver connects |
+------------------------+---------------------------------------------------------------------------------+
| auto_stand             | A boolean to automatically stand the robot after the driver conneccts           |
+------------------------+---------------------------------------------------------------------------------+

.. warning::

  Having the robot power its motors and stand after the driver connects is a physical action and has inherent danger.  Make sure to use these settings with cuation and ensure the area is clear before starting the driver
