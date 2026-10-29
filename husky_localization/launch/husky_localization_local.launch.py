#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Set the path to different files and folders.
    localization_pkg = get_package_share_directory("husky_localization")
    ekf_params_path = join(localization_pkg, 'config', 'husky_gps_localization.yaml')
    rviz_config_file_path = join(localization_pkg, 'config', 'ekf_gps.rviz')
    

    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name="ekf_filter_node_odom",
        output='screen',
        parameters=[ekf_params_path, {"use_sim_time": True}],
        remappings=[("odometry/filtered", "odometry/local")],
    )


    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path]
    )

    husky_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(localization_pkg, "launch", "husky_rsp.launch.py")),
    )
       
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add any actions
    ld.add_action(local_ekf)
    ld.add_action(husky_rsp)

    return ld
