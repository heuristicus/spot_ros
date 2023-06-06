cmake_minimum_required(VERSION 2.8.3)
project(spot_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    scripts/spot_ros
    scripts/examples/world_object_wrapper_test
    scripts/examples/task_wrapper_example
    scripts/examples/hand_pose_example
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY doc DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
