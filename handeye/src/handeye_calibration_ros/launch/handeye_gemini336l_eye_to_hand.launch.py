import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('orbbec_camera'),
            'launch',
            'gemini_330_series.launch.py',
        )),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_left_ir': 'false',
            'enable_right_ir': 'false',
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'enable_d2c_viewer': 'false',
            'enable_frame_sync': 'false',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'color_rotation': LaunchConfiguration('display_rotation'),
        }.items(),
    )

    aruco = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': False,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            'reference_frame': 'camera_color_optical_frame',
            'camera_frame': 'camera_color_optical_frame',
            'marker_frame': LaunchConfiguration('marker_frame'),
            'corner_refinement': LaunchConfiguration('corner_refinement'),
        }],
        remappings=[
            ('/camera_info', '/camera/color/camera_info'),
            ('/image', '/camera/color/image_raw'),
        ],
    )

    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='handeye_image_view',
        output='screen',
        arguments=['/aruco_single/result'],
        condition=IfCondition(LaunchConfiguration('start_rqt')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('marker_id', default_value='582'),
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.10',
            description='Measured ArUco marker side length in metres.'),
        DeclareLaunchArgument(
            'marker_frame', default_value='aruco_marker_frame'),
        DeclareLaunchArgument(
            'corner_refinement',
            default_value='SUBPIX',
            choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX']),
        DeclareLaunchArgument(
            'display_rotation',
            default_value='90',
            choices=['0', '90', '180', '270'],
            description='Clockwise Gemini color image rotation in degrees.'),
        DeclareLaunchArgument(
            'start_rqt',
            default_value='true',
            choices=['true', 'false'],
            description='Open the ArUco detection result in rqt_image_view.'),
        orbbec_launch,
        aruco,
        image_view,
    ])
