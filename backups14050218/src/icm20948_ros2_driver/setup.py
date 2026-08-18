from setuptools import setup
import os
from glob import glob

package_name = 'icm20948_ros2_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 driver for ICM-20948 9-axis IMU sensor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icm20948_node.py = icm20948_ros2_driver.icm20948_node:main',
            'calibrate_imu.py = icm20948_ros2_driver.calibrate_imu:main',
        ],
    },
)
