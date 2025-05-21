from setuptools import setup
import os
from glob import glob

package_name = 'leadshine_motor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your-name',
    maintainer_email='your@email.com',
    description='Leadshine motor control via Modbus',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = leadshine_motor.motor_node:main',
            'motor_simulation_node = leadshine_motor.motor_simulation_node:main',
        ],
    },
)
