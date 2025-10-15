from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lift_robot_pushrod'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot User',
    maintainer_email='user@example.com',
    description='ROS2 package for controlling lift robot pushrod using CH5/CH6 relay states',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pushrod_node = lift_robot_pushrod.pushrod_node:main',
        ],
    },
)
