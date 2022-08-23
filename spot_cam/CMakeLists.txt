cmake_minimum_required(VERSION 3.0.2)
project(spot_cam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  PowerStatus.msg
)

add_service_files(
  FILES
  SetIRColormap.srv
  SetIRMeterOverlay.srv
  SetString.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spot_cam
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   scripts/spot_cam
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)