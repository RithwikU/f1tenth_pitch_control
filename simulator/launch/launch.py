#!/usr/bin/python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('simulator'),
        'config',
        'sim.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    state_publisher_node = Node(
        package='simulator',
        executable='ego_car_state_publisher.py',
        name='bridge',
        parameters=[config]
    )

    # should be replace by state publisher above
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["0", "0", "0", "0", "0", "0", "world", "ego_racecar/base_link"]
    )

    map_node = Node(
        package='simulator',
        namespace='simulator',
        executable='publisher',
        name='publisher'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('simulator'), 'rviz', 'terrain_config.rviz')]
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(get_package_share_directory('simulator'), 'model', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    pitch_controller_node = Node(
        package='simulator',
        namespace='simulator',
        executable='pitch_controller',
        name='pitch_controller'
    )

    ld.add_action(map_node)
    ld.add_action(rviz_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(state_publisher_node)
    ld.add_action(static_transform_node)
    ld.add_action(pitch_controller_node)
    return ld
