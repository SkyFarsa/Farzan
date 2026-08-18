from setuptools import setup

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='My robot package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_odom = my_robot.serial_odom_node:main',  # <-- IMPORTANT
        ],
    },
)

