Spot ROS Computer Setup
=======================

.. note::
    This driver has been tested using the steps below with Spot SDK version 2.3.5.  Any version after this may have unexpected behviour.

The ROS driver was created and tested on Kinetic and Melodic

Setup Spot Core
---------------

If you have a Spot Core, download the latest `Melodic ISO <https://packages.clearpathrobotics.com/stable/images/latest/melodic-bionic/amd64/>`_, set it up with `BalenaEtcher <https://www.balena.io/etcher/>`_, and install it onto the Core

If you are using a Jetson, follow the `Jetson Setup Instructions <https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html>`_

Once your backpack PC is setup, all steps below are to be followed on that PC

Installing Dependencies
-----------------------

.. code:: bash

  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  sudo apt update
  sudo apt install ros-melodic-ros-base

.. code:: bash

  sudo apt update
  sudo apt install -y python3-pip
  pip3 install cython
  pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
  pip3 install empy

Setup Networking
----------------

Replace the `/etc/network/interfaces` file with the one below

.. code:: bash

  auto lo br0 br0:0 br0:1
  iface lo inet loopback

  # Bridge together physical ports on machine, assign standard Clearpath Robot IP.
  iface br0 inet static
    bridge_ports regex (eth.*)|(en.*)
    address 192.168.131.1
    netmask 255.255.255.0
    bridge_maxwait 0

  # Dedicated port for spot
  iface br0:0 inet static
    address 192.168.50.1
    netmask 255.255.255.0

  # Also seek out DHCP IP on those ports, for the sake of easily getting online,
  # maintenance, ethernet radio support, etc.
  iface br0:1 inet dhcp

Building the Driver from Source
-------------------------------

As the driver hasn't been released yet, it must be built from source.  This requires a source workspace on the ROS PC.

.. code:: bash

  mkdir -p ~/catkin_ws/src

Setup the workspace so it knows about your ROS installation

.. code:: bash

  cd ~/catkin_ws/src
  source /opt/ros/melodic/setup.bash
  catkin_init_workspace

Clone the spot_ros repository into the workspace

.. code:: bash

  cd ~/catkin_ws/src
  git clone https://github.com/clearpathrobotics/spot_ros.git
  git clone https://github.com/ros/geometry2 --branch 0.6.5

Use rosdep to install of the necessary dependencies

.. code:: bash

  cd ~/catkin_ws/
  rosdep install --from-paths src --ignore-src -y

Once all the necessary packages are installed, build the packages in the workspace

.. code:: bash

  cd ~/catkin_ws/
  catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

Source your newly built workspace and the packages inside

.. code:: bash

  source ~/catkin_ws/devel/setup.bash
