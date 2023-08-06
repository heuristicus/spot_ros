FROM ros:noetic

RUN DEBIAN_FRONTEND=noninteractive apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y git

# Download the Spot ROS packages
RUN mkdir -p /ros/catkin_ws/src
WORKDIR /ros/catkin_ws/src
# Get the secrets from Github Actions
RUN --mount=type=secret,id=SPOTROS_GIT_BRANCH \
    export SPOTROS_GIT_BRANCH=$(cat /run/secrets/SPOTROS_GIT_BRANCH) && \
    echo $SPOTROS_GIT_BRANCH && \
    git clone -b $SPOTROS_GIT_BRANCH https://github.com/jeremysee2/spot_ros.git /ros/catkin_ws/src/spot_ros

RUN DEBIAN_FRONTEND=noninteractive apt install -y python3-pip git qttools5-dev \
    ros-noetic-tf2-bullet ros-noetic-roslint ros-noetic-pcl-ros ros-noetic-catkin \
    python3-catkin-pkg nano vim ufw net-tools
RUN python3 -m pip install cython bosdyn-client bosdyn-mission bosdyn-api bosdyn-core empy numpy
RUN chmod +x /opt/ros/noetic/setup.sh
RUN /opt/ros/noetic/setup.sh

# Git clone Spot Wrapper
WORKDIR /ros/catkin_ws/src/spot_ros
RUN git clone https://github.com/jeremysee2/spot_wrapper.git /ros/catkin_ws/src/spot_ros/spot_wrapper
RUN pip install -r /ros/catkin_ws/src/spot_ros/spot_wrapper/requirements.txt
RUN pip install -e /ros/catkin_ws/src/spot_ros/spot_wrapper

# Install ROS dependencies
WORKDIR /ros/catkin_ws/

RUN rosdep install --from-paths /ros/catkin_ws/src --ignore-src -y

# Build ROS packages, remember to source the setup.bash file in the same command
WORKDIR /ros/catkin_ws/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make --only-pkg-with-deps spot_driver"
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make"

# Source the new packages
RUN chmod +x /ros/catkin_ws/devel/setup.sh && /ros/catkin_ws/devel/setup.sh

# Setup the environmental variables for including these in the URDF
ARG SPOT_ARM=1
ARG SPOT_PACK=1
ARG SPOT_LIDAR_MOUNT=1
ARG SPOT_VELODYNE=1
ARG SPOT_USERNAME=admin
ARG SPOT_PASSWORD=dummypassword

ARG ROS_MASTER_URI=http://localhost:21311

ENTRYPOINT /bin/bash -c "/ros/catkin_ws/src/spot_ros/deploy/entrypoint.sh"
