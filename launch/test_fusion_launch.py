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
        
        # ROI parameters
        DeclareLaunchArgument('use_roi', default_value='true',
                             description='Enable ROI filtering (true/false)'),
        DeclareLaunchArgument('roi_x', default_value='400',
                             description='ROI top-left X coordinate'),
        DeclareLaunchArgument('roi_y', default_value='200', 
                             description='ROI top-left Y coordinate'),
        DeclareLaunchArgument('roi_width', default_value='800',
                             description='ROI width'),
        DeclareLaunchArgument('roi_height', default_value='600',
                             description='ROI height'),

        # Test LiDAR-Camera fusion node
        Node(
            package='fusion_lidar_camera',
            executable='test_lidar_camera_fusion_node',
            name='test_lidar_camera_fusion_node',
            remappings=[
                ('/livox/lidar', LaunchConfiguration('points_topic')),
                ('/image_raw', LaunchConfiguration('image_topic')),
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
                'roi_height': LaunchConfiguration('roi_height'),
            }]
        )
    ])
