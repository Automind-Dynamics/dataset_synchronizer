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
        parameters=[ekf_params_path, {"use_sim_time": False}],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name="ekf_filter_node_map",
        output='screen',
        parameters=[ekf_params_path, {"use_sim_time": False}],
        remappings=[("odometry/filtered", "odometry/global")],
    )
    
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name="navsat_transform",
        output='screen',
        parameters=[ekf_params_path, {"use_sim_time": False}],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gnss"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
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
    ld.add_action(global_ekf)
    ld.add_action(navsat_transform)
    ld.add_action(husky_rsp)
    ld.add_action(start_rviz)

    return ld
