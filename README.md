# Spot ROS Driver

![CP Spot](cp_spot.jpg)

## Prerequisite
```
pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
```


## Documentation

Check-out the usage and user documentation [HERE](https://heuristicus.github.io/spot_ros)


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
| `SPOT_ARM`                 | `0`                               | If `1`, adds the Spot arm to the URDF                                                             |


# Building Quick-Start

NOTE: please follow the link above for the complete documentation. You will need to configure the networking on both
your computer and the base Spot platform before you can run the driver.

## Install Dependencies

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-ros-base

sudo apt update
sudo apt install -y python3-pip
pip3 install cython
pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
pip3 install empy
```

If using the Velodyne VLP-16 mounted to the standard Spot backpack you will also need the `velodyne_description` and
`velodyne_pointcloud` ROS packages.  These are included as dependencies in the `spot_driver` and `spot_description`
packages respectively, and will be installed if you use `rosdep install ...` in your workspace.

The `teleop_joy` package is also included as a dependency to allow Spot to be controlled using a bluetooth or USB
game controller.


## Building for Melodic

Please note that the Spot SDK uses Python3, which is not officially supported by ROS Melodic.  If you encounter an error
of this form:

```bash
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
```

when launching the driver, please follow these steps:

1.  `rm -rf devel/ build/ install/` -- this will remove any old build artifacts from your workspace

2. `git clone https://github.com/ros/geometry2 --branch 0.6.5` into your `src` folder

3. rebuild your workspace with

```bash
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```

Or if you are using an Nvidia Jetson:

```bash
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
```

4. re-run `source devel/setup.bash`

5. start the driver with `roslaunch spot_driver driver.launch`

# MoveIt simulation of Spot's arm

Can be found in other repo: https://github.com/estherRay/Spot-Arm
