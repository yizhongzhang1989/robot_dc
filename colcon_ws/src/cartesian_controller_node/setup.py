from setuptools import find_packages, setup

package_name = 'monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/zero_force_control.launch.py', 
             'launch/joystick_force_control.launch.py', 
             'launch/joystick_compliance_control.launch.py', 
             'launch/joystick_calibration.launch.py', 
             'launch/calibration_data_recorder.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Robot monitoring node with UDP data collection and rosbag recording',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zero_force_control_node = monitor.zero_force_control_node:main',
            'joystick_force_control_node = monitor.joystick_force_control_node:main',
            'joystick_compliance_control_node = monitor.joystick_compliance_control_node:main',
            'joystick_calibration_node = monitor.joystick_calibration_node:main',
            'calibration_data_recorder = monitor.calibration_data_recorder:main',
        ],
    },
)
