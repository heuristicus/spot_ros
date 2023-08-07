cmake_minimum_required(VERSION 3.0.2)
project(spot_cam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  spot_msgs
  spot_driver
  geometry_msgs
  message_generation
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  BITStatus.msg
  Degradation.msg
  PowerStatus.msg
  PTZDescription.msg
  PTZDescriptionArray.msg
  PTZLimits.msg
  PTZState.msg
  PTZStateArray.msg
  StreamParams.msg
  StringMultiArray.msg
  Temperature.msg
  TemperatureArray.msg
)

add_service_files(
  FILES
  LoadSound.srv
  LookAtPoint.srv
  PlaySound.srv
  SetBool.srv
  SetFloat.srv
  SetIRColormap.srv
  SetIRMeterOverlay.srv
  SetPTZState.srv
  SetStreamParams.srv
  SetString.srv
)

add_action_files(
  FILES
  LookAtPoint.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  spot_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spot_cam
  CATKIN_DEPENDS message_runtime std_msgs spot_msgs geometry_msgs
#  DEPENDS system_lib
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   scripts/spot_cam
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
