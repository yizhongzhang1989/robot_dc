from setuptools import setup

package_name = 'leadshine_motor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (f'share/{package_name}/config', ['config/motor1.yaml']),
        (f'share/{package_name}/launch', ['launch/jog_motor.launch.py']),
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
            'jog_node = leadshine_motor.jog_node:main',
        ],
    },
)
