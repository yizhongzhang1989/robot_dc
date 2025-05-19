from setuptools import setup
import os
from glob import glob

package_name = 'modbus_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Modbus RTU shared interface node for multiple device clients',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modbus_manager_node = modbus_driver.modbus_manager_node:main',
            'modbus_client_tester = modbus_driver.modbus_client_tester:main',
        ],
    },
)
