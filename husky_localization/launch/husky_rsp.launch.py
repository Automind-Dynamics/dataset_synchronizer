#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression, Command


def generate_launch_description():

    # Set the path to different files and folders.
    description_pkg = get_package_share_directory("husky_description")
    xacro_path = join(description_pkg, 'urdf', 'husky_raw.urdf.xacro')
    

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                    ])},
                    {"use_sim_time": False}]
    )
       
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add any actions
    ld.add_action(start_robot_state_publisher)

    return ld
