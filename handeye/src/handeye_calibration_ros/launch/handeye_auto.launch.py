import os
from ament_index_python.packages import get_package_share_directory
from handeye_calibration_ros.hardware import acquire_instance_lock, require_can_interface, select_camera, usb_devices
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


_INSTANCE_LOCK = None


def launch_setup(context):
    global _INSTANCE_LOCK
    _INSTANCE_LOCK = acquire_instance_lock()
    requested_camera = LaunchConfiguration('camera').perform(context)
    requested_mode = LaunchConfiguration('mode').perform(context)
    camera, auto_mode, device = select_camera(
        usb_devices(), requested_camera=requested_camera)
    mode = auto_mode if requested_mode == 'auto' else requested_mode
    expected_mode = 'eye_in_hand' if camera == 'realsense' else 'eye_to_hand'
    if mode != expected_mode:
        raise RuntimeError(
            f'{camera} must use {expected_mode}; requested mode was {mode}.')
    can_port = LaunchConfiguration('can_port').perform(context)
    require_can_interface(
        can_port, devices=usb_devices(), auto_activate=True)

    config_file = LaunchConfiguration('charuco_config').perform(context)
    with open(config_file, 'r', encoding='utf-8') as stream:
        detector_config = yaml.safe_load(stream)['charuco_detector']['ros__parameters']

    if camera == 'realsense':
        camera_launch = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': False,
                'enable_infra': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.color_profile': '1280,720,30',
                'initial_reset': False,
            }],
        )
        image_topic = '/camera/camera/color/image_raw'
        camera_info_topic = '/camera/camera/color/camera_info'
    else:
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch', 'gemini_330_series.launch.py')),
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
                'color_width': '1280',
                'color_height': '720',
                'color_fps': '30',
            }.items(),
        )
        image_topic = '/camera/color/image_raw'
        camera_info_topic = '/camera/color/camera_info'

    piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        parameters=[{
            'can_port': can_port,
            'auto_enable': False,
            'gripper_exist': True,
        }],
        remappings=[('joint_ctrl_single', '/joint_states')],
    )
    flange_node = None
    robot_topic = '/end_pose_stamped'
    robot_base_frame = 'base'
    robot_end_frame = 'gripper'
    if mode == 'eye_in_hand':
        robot_topic = '/flange_pose_stamped'
        robot_base_frame = 'base_link'
        robot_end_frame = 'link6'
        flange_node = Node(
            package='handeye_calibration_ros',
            executable='flange_pose',
            name='flange_pose',
            output='screen',
            parameters=[{
                'urdf_path': os.path.join(
                    get_package_share_directory('piper_description'),
                    'urdf', 'piper_description.urdf'),
                'joint_topic': '/joint_states_feedback',
                'pose_topic': robot_topic,
                'base_frame': robot_base_frame,
                'flange_frame': robot_end_frame,
            }],
        )
    detector_node = Node(
        package='handeye_calibration_ros',
        executable='charuco_detector',
        name='charuco_detector',
        output='screen',
        parameters=[
            config_file,
            {
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
            },
        ],
    )
    session_node = Node(
        package='handeye_calibration_ros',
        executable='handeye_session',
        name='handeye_session',
        output='screen',
        parameters=[
            config_file,
            {
                'camera': camera,
                'mode': mode,
                'robot_topic': robot_topic,
                'robot_base_frame': robot_base_frame,
                'robot_end_frame': robot_end_frame,
                'marker_topic': detector_config.get('pose_topic', '/charuco/pose'),
                'result_image_topic': detector_config.get(
                    'result_image_topic', '/charuco/result'),
                'result_save_path': LaunchConfiguration('result_save_path'),
                'board_columns': detector_config['columns'],
                'board_rows': detector_config['rows'],
                'board_dictionary': detector_config['dictionary'],
                'board_square_length_m': detector_config['square_length'],
                'board_marker_length_m': detector_config['marker_length'],
                'ui_mode': 'rqt',
            },
        ],
    )

    def shutdown_unless_already_stopping(reason):
        def on_exit(event, launch_context):
            if launch_context.is_shutdown:
                return []
            return [EmitEvent(event=Shutdown(reason=reason))]
        return on_exit

    shutdown_on_session_exit = RegisterEventHandler(OnProcessExit(
        target_action=session_node,
        on_exit=shutdown_unless_already_stopping('Hand-eye session finished'),
    ))
    shutdown_on_piper_exit = RegisterEventHandler(OnProcessExit(
        target_action=piper_node,
        on_exit=shutdown_unless_already_stopping('Piper driver stopped'),
    ))
    summary = LogInfo(msg=(
        f'Auto-selected {device["product"]}: {mode}; USB speed '
        f'{device["speed_mbps"]:.0f} Mbps; board config {config_file}'))
    rqt_panel = ExecuteProcess(
        cmd=[
            'rqt', '--force-discover', '--standalone',
            'handeye_calibration_ros.handeye_rqt.HandeyeRqtPlugin'],
        output='screen',
    )
    actions = [summary, camera_launch, piper_node]
    if flange_node is not None:
        actions.append(flange_node)
    actions.extend([
        detector_node, session_node, rqt_panel,
        shutdown_on_session_exit, shutdown_on_piper_exit,
    ])
    return actions


def generate_launch_description():
    package_share = get_package_share_directory('handeye_calibration_ros')
    return LaunchDescription([
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument(
            'camera', default_value='auto',
            choices=['auto', 'realsense', 'orbbec']),
        DeclareLaunchArgument(
            'mode', default_value='auto',
            choices=['auto', 'eye_in_hand', 'eye_to_hand']),
        DeclareLaunchArgument(
            'charuco_config',
            default_value=os.path.join(package_share, 'config', 'charuco.yaml')),
        DeclareLaunchArgument(
            'result_save_path',
            default_value=os.path.expanduser('~/gy_ws/handeye/results')),
        OpaqueFunction(function=launch_setup),
    ])
