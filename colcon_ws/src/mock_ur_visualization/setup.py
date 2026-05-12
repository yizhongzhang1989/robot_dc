from setuptools import setup
import os
from glob import glob

package_name = 'mock_ur_visualization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_dc',
    maintainer_email='dev@example.com',
    description='RViz visualization driven by direct URScript polling.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_publisher = mock_ur_visualization.joint_publisher:main',
        ],
    },
)
