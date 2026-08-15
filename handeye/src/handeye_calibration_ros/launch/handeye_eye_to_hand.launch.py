from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('handeye_calibration_ros'),
                'launch', 'handeye_auto.launch.py',
            ])),
            launch_arguments={
                'camera': 'orbbec',
                'mode': 'eye_to_hand',
            }.items(),
        )
    ])
