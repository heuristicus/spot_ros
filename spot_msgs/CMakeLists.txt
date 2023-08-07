cmake_minimum_required(VERSION 3.0.2)
project(spot_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  BatteryStateArray.msg
  BehaviorFault.msg
  EStopStateArray.msg
  FootStateArray.msg
  LeaseArray.msg
  LeaseOwner.msg
  Metrics.msg
  MobilityParams.msg
  SystemFault.msg
  WiFiState.msg
  BatteryState.msg
  BehaviorFaultState.msg
  EStopState.msg
  Feedback.msg
  FootState.msg
  Lease.msg
  LeaseResource.msg
  PowerState.msg
  SystemFaultState.msg
  DockState.msg
  ObstacleParams.msg
  TerrainParams.msg
  TerrainState.msg
  SpotCheckDepth.msg
  SpotCheckHipROM.msg
  SpotCheckKinematic.msg
  SpotCheckLoadCell.msg
  SpotCheckPayload.msg
  AprilTagProperties.msg
  FrameTreeSnapshot.msg
  ParentEdge.msg
  ImageCapture.msg
  ImageProperties.msg
  ImageSource.msg
  WorldObject.msg
  WorldObjectArray.msg
)

add_service_files(
  FILES
  DownloadGraph.srv
  ListGraph.srv
  GraphCloseLoops.srv
  SetLocomotion.srv
  SetSwingHeight.srv
  SetVelocity.srv
  ClearBehaviorFault.srv
  ArmJointMovement.srv
  ConstrainedArmJointMovement.srv
  GripperAngleMove.srv
  ArmForceTrajectory.srv
  HandPose.srv
  SetObstacleParams.srv
  SetTerrainParams.srv
  PosedStand.srv
  Dock.srv
  GetDockState.srv
  SpotCheck.srv
  Grasp3d.srv
  NavigateInit.srv
)

add_action_files(
  FILES
  Dock.action
  NavigateTo.action
  NavigateRoute.action
  PoseBody.action
  Trajectory.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
