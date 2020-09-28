from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('spot_viz')

    return LaunchDescription([
        Node(
            package='rviz',
            namespace='rviz',
            executable='rviz',
            name='rviz',
            arguments=['-d', os.path.join(bringup_dir,'rviz', 'robot.rviz"')]
        )
    ])
