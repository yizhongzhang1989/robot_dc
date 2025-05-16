from setuptools import setup

package_name = 'modbus_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Reusable Modbus RTU interface for ROS 2',
    license='MIT',
    tests_require=['pytest'],
)
