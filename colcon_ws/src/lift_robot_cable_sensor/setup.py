from setuptools import find_packages, setup

package_name = 'lift_robot_cable_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cable_sensor_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Lift robot cable sensor node for reading sensor data via Modbus',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cable_sensor_node = lift_robot_cable_sensor.cable_sensor_node:main',
        ],
    },
)
