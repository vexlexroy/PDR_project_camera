import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    video_arg = DeclareLaunchArgument(
        'camera_source',
        default_value= "'0'",  # camera id or path to video file
        description='Video device index or path to video file'
    )

    camera_pub = Node(
        package='camera_test_gt',        # ← your camera node’s package name
        executable='camera_pub_node',         # ← your camera node’s executable
        name='camera_pub_node',
        output='screen',
        parameters=[{
            'camera_source': LaunchConfiguration('camera_source')
        }],
        remappings=[
            ('/camera/image_raw', '/gt_test/image_raw')  # example: publish to /gt_test/image_raw
        ]
    )

    gt_tester = Node(
        package='camera_test_gt',     # ← your package name
        executable='gt_test_detector_node',      # ← the name in your setup.py entry_point
        name='gt_test_node',
        output='screen',
        remappings=[
            ('/capture', '/gt_test/image_raw'),   # subscribe to camera pub output
            ('/camera_pose', '/gt_test/camera_pose')       # pose output topic
        ],
        parameters=[
            {'marker_size_mm': 10.0},
            {'camera_matrix': [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]},
            {'distortion_matrix': [0.0, 0.0, 0.0, 0.0, 0.0]},
        ]
    )

    return LaunchDescription([
        video_arg,
        camera_pub,
        gt_tester
    ])
