from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_labeling_service'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Yizhong Zhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='ROS2 wrapper for image labeling web service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_labeling_node = image_labeling_service.image_labeling_node:main',
        ],
    },
)
