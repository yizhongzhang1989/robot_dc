from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_arm_teleop',
    maintainer_email='yizhongzhang1989@example.com',
    description='Teleop control for robot arm using joystick',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_arm_teleop = robot_arm_teleop.robot_arm_teleop_node:main',
        ],
    },
)
