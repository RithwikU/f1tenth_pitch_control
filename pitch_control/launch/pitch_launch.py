import os
import rospkg

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = os.path.join(
        get_package_share_directory('pitch-control'),
        'waypoints', "")

    return LaunchDescription([
        Node(
            package='pitch_control',
            executable='pitch_control_node',
            name='pitch_control_node',
            output='screen',
        ),
        Node(
            package='pitch_control',
            executable='straight_pub_node',
            name='straight_pub_node',
            output='screen',
        ),
    ])