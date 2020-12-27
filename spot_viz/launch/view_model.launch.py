from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('spot_viz')
    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            namespace='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            remappings=[
                ('/joint_state_publisher_gui/joint_states', '/joint_states'),
                ('/joint_state_publisher_gui/robot_description', '/robot_description'),
                ('/joint_state_publisher_gui/parameter_events', '/parameter_events')
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('spot_description'),
                             'launch',
                             'description.launch.py')
            )
        ),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup_dir,'rviz', 'robot.rviz"')],
        )
    ])
