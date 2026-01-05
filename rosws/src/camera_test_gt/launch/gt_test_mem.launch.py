import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('camera_test_gt')
    config_file = os.path.join(pkg_share, 'config', 'params_gt_test.yaml')


    # video_arg = DeclareLaunchArgument(
    #     'camera_source',
    #     default_value= "'0'",  # camera id or path to video file
    #     description='Video device index or path to video file'
    # )

    camera_pub = Node(
        package='camera_test_gt',        #camera node’s package name
        executable='camera_pub_node',         #camera node’s executable
        name='camera_pub_node',
        output='screen',
        parameters=[config_file], # <-- load from YAML
        remappings=[
            ('/camera/image_raw', '/gt_test/image_raw')
        ]
    )

    gt_tester = Node(
        package='camera_test_gt',     # package name
        executable='gt_test_detector_memory_node',
        name='gt_test_node',
        output='screen',
        remappings=[
            ('/capture', '/gt_test/image_raw'),   # subscribe to camera pub output
            ('/camera_pose', '/gt_test/camera_pose'),       # pose output topic
            ('/marker_image', '/gt_test/marker_image'),
            ('/camera_path', '/gt_test/camera_path')
        ],
        parameters=[config_file] # <-- load from YAML
    )

    return LaunchDescription([
        # video_arg,
        camera_pub,
        gt_tester
    ])
