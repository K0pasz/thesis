from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch PointCloudSaver node
        Node(
            package='pointcloud_saver',
            executable='pointcloud_saver',
            name='pointcloud_saver',
            output='screen',
            parameters=[
                {'ply_file_path': 'sparse_pc.ply'}  # Set PLY file path if needed
            ]
        ),
        
        # Launch KeyframeImageSaver node
        Node(
            package='keyframe_saver',
            executable='keyframe_saver',
            name='keyframe_saver',
            output='screen',
            parameters=[
                {'save_directory': 'keyframes'}  # Set directory for saving images and poses
            ]
        )
    ])
