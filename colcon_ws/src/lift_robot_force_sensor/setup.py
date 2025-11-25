from setuptools import setup

package_name = 'lift_robot_force_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lift_robot_force_sensor_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Lift robot force sensor reader node (single channel, Modbus FC03).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'force_sensor_node = lift_robot_force_sensor.force_sensor_node:main',
        ],
    },
)
