from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'duco_robot_arm'

# Find all packages including subpackages
packages = find_packages(exclude=['test'])
# Add the gen_py and lib subpackages manually
packages.extend([
    'duco_robot_arm.gen_py',
    'duco_robot_arm.gen_py.robot',
    'duco_robot_arm.lib',
    'duco_robot_arm.lib.thrift',
    'duco_robot_arm.lib.thrift.transport',
    'duco_robot_arm.lib.thrift.protocol',
    'duco_robot_arm.lib.thrift.server',
])

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'duco_robot_arm_node = duco_robot_arm.duco_robot_arm_node:main',
        ],
    },
)
