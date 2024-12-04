## 1. Quick Start

You don't need a physical robot to run the following demos.

### 1.1. Walking demo in RVIZ:

#### 1.1.1. Run the base driver:

    roslaunch spot_champ_config bringup.launch rviz:=true

#### 1.1.2. Run the teleop node:

    roslaunch champ_teleop teleop.launch

If you want to use a [joystick](https://www.logitechg.com/en-hk/products/gamepads/f710-wireless-gamepad.html) add joy:=true as an argument.

### 1.2. SLAM demo:

#### 1.2.1. Run the Gazebo environment:

    roslaunch spot_champ_config gazebo.launch

#### 1.2.2. Run gmapping package and move_base:

    roslaunch spot_champ_config slam.launch rviz:=true

To start mapping:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

  ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/slam.gif)

- Save the map by running:

      roscd spot_champ_config/maps
      rosrun map_server map_saver

### 1.3. Autonomous Navigation:

#### 1.3.1. Run the Gazebo environment:

    roslaunch spot_champ_config gazebo.launch

#### 1.3.2. Run amcl and move_base:

    roslaunch spot_champ_config navigate.launch rviz:=true

To navigate:

- Click '2D Nav Goal'.
- Click and drag at the position you want the robot to go.

  ![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/navigation.gif)

#### 1.4.1 Spawning multiple robots in Gazebo

Run Gazebo and default simulation world:

    roslaunch champ_gazebo spawn_world.launch

You can also load your own world file by passing your world's path to 'gazebo_world' argument:

    roslaunch champ_gazebo spawn_world.launch gazebo_world:=<path_to_world_file>

Spawning a robot:

    roslaunch spot_champ_config spawn_robot.launch robot_name:=<unique_robot_name> world_init_x:=<x_position> world_init_y:=<y_position>

- Every instance of the spawned robot must have a unique robot name to prevent the topics and transforms from clashing.

---

:exclamation: _This is not an official product from the robot's company/author._
