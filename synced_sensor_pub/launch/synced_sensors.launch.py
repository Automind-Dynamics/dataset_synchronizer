from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('slop', default_value='0.05'),
        DeclareLaunchArgument('queue_size', default_value='10'),
        DeclareLaunchArgument('restamp_to_rgb', default_value='false'),
    ]

    # 1) Synchronizer: 3 compressed/raw inputs -> 3 synced outputs
    sync_node = Node(
        package='synced_sensor_pub',
        executable='sync_node',
        name='sync_node',
        output='screen',
        parameters=[{
            'depth_topic': '/zedx_front/zed_node/depth/depth_registered/compressedDepth',
            'rgb_topic': '/zedx_front/zed_node/rgb/color/rect/image/compressed',
            'lidar_topic': '/ouster/points',
            'synced_depth_topic': '/synced/depth/compressedDepth',
            'synced_rgb_topic': '/synced/rgb/compressed',
            'synced_lidar_topic': '/synced/points',
            'slop': LaunchConfiguration('slop'),
            'queue_size': LaunchConfiguration('queue_size'),
            'restamp_to_rgb': LaunchConfiguration('restamp_to_rgb'),
        }],
    )

    # 2) image_transport republish: synced compressed RGB -> raw Image
    rgb_republish = Node(
        package='image_transport',
        executable='republish',
        name='rgb_republish',
        output='screen',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/synced/rgb/compressed'),
            ('out', '/synced/rgb/image_raw'),
        ],
    )

    # 3) image_transport republish: synced compressedDepth -> raw depth Image
    depth_republish = Node(
        package='image_transport',
        executable='republish',
        name='depth_republish',
        output='screen',
        arguments=['compressedDepth', 'raw'],
        remappings=[
            ('in/compressedDepth', '/synced/depth/compressedDepth'),
            ('out', '/synced/depth/image_raw'),
        ],
    )

    return LaunchDescription(args + [sync_node, rgb_republish, depth_republish])
