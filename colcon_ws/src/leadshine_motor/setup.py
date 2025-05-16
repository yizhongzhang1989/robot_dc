from setuptools import setup

package_name = 'leadshine_motor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/jog_motor.launch.py']),
        ('share/' + package_name + '/config', ['config/motor1.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 package to control Leadshine motor using Modbus RTU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jog_node = leadshine_motor.jog_node:main',
        ],
    },
)
