from setuptools import setup

package_name = 'lift_robot_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', ['web/index.html']),
        ('share/' + package_name + '/launch', ['launch/lift_robot_web.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Lift robot simple web viewer for cable sensor data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['server = lift_robot_web.server:main']},
)
