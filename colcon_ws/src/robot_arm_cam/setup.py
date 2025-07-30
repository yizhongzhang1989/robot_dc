from setuptools import setup
import os
from glob import glob

package_name = 'robot_arm_cam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yizhongzhang1989',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Robot arm camera node for RTSP streaming and control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_arm_cam = robot_arm_cam.robot_arm_cam:main',
        ],
    },
)
