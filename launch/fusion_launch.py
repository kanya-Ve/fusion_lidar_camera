from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('points_topic', default_value='/livox/lidar'),
        # DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw/downsampled'),
        DeclareLaunchArgument('colored_points_topic', default_value='/colored_pointcloud'),
        DeclareLaunchArgument('projected_image_topic', default_value='/projected_image'),
        
        # ROI parameters
        DeclareLaunchArgument('use_roi', default_value='false'),
        DeclareLaunchArgument('roi_x', default_value='0'),
        DeclareLaunchArgument('roi_y', default_value='0'),
        DeclareLaunchArgument('roi_width', default_value='1920'),
        DeclareLaunchArgument('roi_height', default_value='1080'),

        # LiDAR-Camera fusion node with ROI support
        Node(
            package='fusion_lidar_camera',
            executable='lidar_camera_fusion_node',
            name='fusion_lidar_camera_node',
            remappings=[
                ('/livox/lidar', LaunchConfiguration('points_topic')),
                ('/image_raw/downsampled', LaunchConfiguration('image_topic')),
                ('/colored_pointcloud', LaunchConfiguration('colored_points_topic')),
                ('/projected_image', LaunchConfiguration('projected_image_topic')),
            ],
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'use_roi': LaunchConfiguration('use_roi'),
                'roi_x': LaunchConfiguration('roi_x'),
                'roi_y': LaunchConfiguration('roi_y'),
                'roi_width': LaunchConfiguration('roi_width'),
                'roi_height': LaunchConfiguration('roi_height')
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
