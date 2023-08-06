#!/bin/bash
source /ros/catkin_ws/devel/setup.sh && \
    roslaunch spot_driver driver.launch username:=admin password:=$SPOT_PASSWORD
