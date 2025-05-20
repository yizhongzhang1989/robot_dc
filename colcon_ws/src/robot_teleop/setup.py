from setuptools import setup

package_name = 'robot_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick_teleop_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Joystick-based teleoperation for the robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_teleop = robot_teleop.joystick_teleop_node:main',
        ],
    },
)
