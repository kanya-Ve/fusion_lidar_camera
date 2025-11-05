from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('points_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('colored_points_topic', default_value='/colored_pointcloud'),
        DeclareLaunchArgument('projected_image_topic', default_value='/projected_image'),

        # Simple LiDAR-Camera fusion node (without camera_info)
        Node(
            package='fusion_lidar_camera',
            executable='simple_lidar_camera_fusion_node',
            name='simple_lidar_camera_fusion_node',
            remappings=[
                ('/livox/lidar', LaunchConfiguration('points_topic')),
                ('/image_raw', LaunchConfiguration('image_topic')),
                ('/colored_pointcloud', LaunchConfiguration('colored_points_topic')),
                ('/projected_image', LaunchConfiguration('projected_image_topic')),
            ],
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='usu_to_livox_tf',
            arguments=[
                '0.0975', '0.0', '-0.047',   # x, y, z [m] (例: LiDARの位置)
                '0.0', '0', '3.1415926535',      # roll, pitch, yaw [rad] (例: LiDARの向き)
                'usu',               # 親フレーム
                'livox_frame'           # 子フレーム
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_to_camera_tf',
            arguments=[
                '0.016612', '0.003873', '-0.068737',   # x, y, z [m] (例: LiDARの位置)
                '1.566196', '0.011168', '1.306806',      # roll, pitch, yaw [rad] (例: LiDARの向き)
                'livox_frame',               # 親フレーム
                'camera_frame'           # 子フレーム
            ]
        )
    ])
