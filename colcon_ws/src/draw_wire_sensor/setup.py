from setuptools import setup

package_name = 'draw_wire_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml','README.md']),
        ('share/' + package_name + '/launch', ['launch/draw_wire_sensor.launch.py']),
        ('share/' + package_name + '/test', ['test/test_draw_wire_sensor.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Draw wire sensor node publishing Modbus data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['draw_wire_sensor_node = draw_wire_sensor.draw_wire_sensor_node:main']},
)
