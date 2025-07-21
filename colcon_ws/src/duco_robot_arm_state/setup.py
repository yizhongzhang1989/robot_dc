from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'duco_robot_arm_state'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='ROS2 package for monitoring Duco robot arm state via TCP port 2001',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'duco_robot_arm_state_node = duco_robot_arm_state.duco_robot_arm_state_node:main',
        ],
    },
)
