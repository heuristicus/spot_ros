Spot ROS Computer Setup
=======================

Building the Driver from Source
-------------------------------

The driver is not released to the ROS package servers, so it must be built from source. This requires a source workspace on the ROS PC. You can add it to an existing workspace or follow the instructions below to create a new one.

.. code:: bash

  mkdir -p ~/spot_ws/src

Setup the workspace so it knows about your ROS installation

.. code:: bash

  cd ~/spot_ws/src
  source /opt/ros/noetic/setup.bash
  catkin_init_workspace

Clone the spot_ros repository into the workspace

.. code:: bash

  cd ~/spot_ws/src
  git clone https://github.com/heuristicus/spot_ros.git

Use rosdep to install of the necessary dependencies

.. code:: bash

  cd ~/catkin_ws/
  rosdep install --from-paths spot_driver spot_msgs spot_viz spot_description --ignore-src -y

Once all the necessary packages are installed, build the packages in the workspace

.. code:: bash

  cd ~/spot_ws/
  catkin build

Source your newly built workspace and the packages inside

.. code:: bash

  source ~/spot_ws/devel/setup.bash

Adding to the URDF
------------------

.. warning::

  When adding payloads, you should always add them to the robot's configuration through the `admin panel <https://support.bostondynamics.com/s/article/Payload-configuration-requirements>`_. Adding only the URDF does not add the payload to spot's built in collision avoidance.

You can add to the URDF of the robot by using the ``SPOT_URDF_EXTRAS`` environment variable, which must be set in the
terminal from which the driver is run.. This should point to a urdf file which has an object where the parent is one
of the frames on the robot.

It might look something like below.

.. code:: xml

    <?xml version="1.0"?>
    <robot name="spot_frontier" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="$(find my_payload_package)/urdf/my_payload.xacro"/>
      <xacro:my_payload parent="real_rear_rail"/>
    </robot>

Building for Melodic
--------------------

.. note::

  If you can, consider using ROS noetic instead, which does not have these build issues.

Please note that the Spot SDK uses Python3, which is not officially supported by ROS Melodic.  If you encounter an error
of this form:

.. code:: bash

    Traceback (most recent call last):
      File "/home/administrator/catkin_ws/src/spot_ros/spot_driver/scripts/spot_ros", line 3, in <module>
        from spot_driver.spot_ros import SpotROS
      File "/home/administrator/catkin_ws/src/spot_ros/spot_driver/src/spot_driver/spot_ros.py", line 19, in <module>
        import tf2_ros
      File "/opt/ros/melodic/lib/python2.7/dist-packages/tf2_ros/__init__.py", line 38, in <module>
        from tf2_py import *
      File "/opt/ros/melodic/lib/python2.7/dist-packages/tf2_py/__init__.py", line 38, in <module>
        from ._tf2 import *
    ImportError: dynamic module does not define module export function (PyInit__tf2)

When launching the driver, please follow these steps:

1.  ``rm -rf devel/ build/ install/`` -- this will remove any old build artifacts from your workspace

2. ``git clone https://github.com/ros/geometry2 --branch 0.6.5`` into your ``src`` folder

3. rebuild your workspace with

.. code:: bash

    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so


Or if you are using an Nvidia Jetson:

.. code:: bash

    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so


4. re-run ``source devel/setup.bash``

5. start the driver with ``roslaunch spot_driver driver.launch``
