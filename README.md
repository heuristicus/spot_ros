# Spot ROS Driver

![CP Spot](cp_spot.jpg)

If you prefer ROS2, a [ROS2 version](https://github.com/bdaiinstitute/spot_ros2/) of this driver is also available.

## Prerequisites

You should be using ROS noetic. ROS melodic can also be used but may require modifications to build correctly.

You must install the Boston Dynamics SDK on any machine that will run the driver.
```
pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
```

## Quick start
### Installing the packages

Go to the source directory of your ROS workspace and clone this repository with
```bash
git clone git@github.com:heuristicus/spot_ros.git
```

Then, initialise the submodule for the wrapper we use to interact with the Boston Dynamics SDK

```bash
cd spot_ros
git submodule init
git submodule update
```

Then, install the python package containing the wrapper

```bash
pip3 install -e spot_wrapper 
```
Build the ROS packages 

```bash
catkin build spot_driver spot_viz
```

Finally, remember to source your workspace.

### Connecting to the robot

To test functionality, it's easiest to connect to the robot via wifi. For actual operation it is recommended to connect to the robot directly through payload ports for higher bandwidth.

Connect to the robot's wifi network, usually found at SSID `spot-BD-xxxxxxxx`. The password for the network is found in the robot's battery compartment.

Once connected, verify that you can ping the robot with `ping 192.168.80.3`.

Start a roscore on your machine with `roscore`.

Run the driver with the username and password for the robot, again found in the battery compartment

```bash
roslaunch spot_driver driver.launch username:=user password:=[your-password] hostname:=192.168.80.3
```

You can then view and control the robot from the rviz interface by running

```bash
roslaunch spot_viz view_robot.launch
```
## Documentation

More detailed documentation can be found [here](https://heuristicus.github.io/spot_ros)

# MoveIt simulation of Spot's arm

Can be found [here](https://github.com/estherRay/Spot-Arm).
