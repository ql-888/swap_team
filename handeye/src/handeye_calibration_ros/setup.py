from setuptools import setup
import os
import glob
from glob import glob


package_name = 'handeye_calibration_ros'

setup(
    name=package_name,
    version='0.0.0',  
    packages=[package_name], 
    install_requires=['setuptools', 'rclpy'],
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), ['plugin.xml']),
    ],
    zip_safe=True,
    author='agilex',
    author_email='agilex@todo.todo',
    description='TODO: Package description',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'handeye_calibration = handeye_calibration_ros.handeye_calibration:main',
            'charuco_detector = handeye_calibration_ros.charuco_detector:main',
            'handeye_session = handeye_calibration_ros.handeye_session:main',
            'handeye_rqt = handeye_calibration_ros.handeye_rqt:main',
            'flange_pose = handeye_calibration_ros.flange_pose:main',
        ],
    }
)
