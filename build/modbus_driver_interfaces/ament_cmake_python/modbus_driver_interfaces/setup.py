from setuptools import find_packages
from setuptools import setup

setup(
    name='modbus_driver_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('modbus_driver_interfaces', 'modbus_driver_interfaces.*')),
)
