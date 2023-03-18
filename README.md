# Spot ROS Driver

![CP Spot](cp_spot.jpg)

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


## Environment Variables

This driver supports configuration via environment variables.  These can either be manually set in the terminal before
starting the driver, or added to $HOME/.bashrc as-desired.  The table below lists the available variables and their
default values:

| Variable                   | Default Value                     | Description                                                                                |
|----------------------------|-----------------------------------|--------------------------------------------------------------------------------------------|
| `SPOT_PACK`                | `0`                               | If `1`, enables the standard ROS backpack accessory and adds it to the URDF                |
| `SPOT_LIDAR_MOUNT`         | `0`                               | If `1`, adds the Lidar mount to the backpack. Requires `SPOT_PACK` to be `1`               |
| `SPOT_VELODYNE`            | `0`                               | If `1`, adds the a VLP-16 sensor to the lidar mount. Requires `SPOT_LIDAR_MOUNT` to be `1` |
| `SPOT_VELODYNE_AUTOLAUNCH` | `1`                               | If `1` and `SPOT_VELODYNE` is also 1, the VLP16 ROS node will start automatically          |
| `SPOT_VELODYNE_XYZ`        | `0 0 0`                           | XYZ offset for the VLP-16 from the backpack lidar mount                                    |
| `SPOT_VELODYNE_RPY`        | `0 0 0`                           | RPY offset for the VLP-16 from the backpack lidar mount                                    |
| `SPOT_VELODYNE_HOST`       | `192.168.131.20`                  | IP address of the VLP-16 sensor                                                            |
| `SPOT_URDF_EXTRAS`         | `empty.urdf`                      | Optional URDF file to add additional joints and links to the robot                         |
| `SPOT_JOY_DEVICE`          | `/dev/input/js0`                  | The Linux joypad input device used by the `joy_teleop` node                                |
| `SPOT_JOY_CONFIG`          | `spot_control/config/teleop.yaml` | Joypad button/axis configuration file for `joy_teleop`                                     |
| `SPOT_ARM`                 | `0`                               | If `1`, adds the Spot arm to the URDF                                                      |

# MoveIt simulation of Spot's arm

Can be found [here](https://github.com/estherRay/Spot-Arm).