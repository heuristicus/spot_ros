cmake_minimum_required(VERSION 3.5)
project(spot_driver_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES 'Clang')
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/BatteryState.msg"
    "msg/BatteryStateArray.msg"
    "msg/BehaviorFault.msg"
    "msg/EStopStateArray.msg"
    "msg/FootStateArray.msg"
    "msg/FramePose.msg"
    "msg/LeaseArray.msg"
    "msg/LeaseOwner.msg"
    "msg/Metrics.msg"
    "msg/SystemFault.msg"
    "msg/WiFiState.msg"
    "msg/BehaviorFaultState.msg"
    "msg/EStopState.msg"
    "msg/Feedback.msg"
    "msg/FootState.msg"
    "msg/Lease.msg"
    "msg/LeaseResource.msg"
    "msg/PowerState.msg"
    "msg/SystemFaultState.msg"
    "srv/CreateFrame.srv"
    "srv/SetOrientation.srv"
    "srv/SetParameter.srv"
    "srv/StrikeAPose.srv"
    "srv/Turn.srv"
    "srv/Walk.srv"
    "srv/WalkTo.srv"
    "srv/WalkToInFrame.srv"
    "srv/WalkToRelative.srv"
    DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs builtin_interfaces)

ament_package()