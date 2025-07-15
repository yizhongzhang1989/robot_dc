from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
        (os.path.join('share', package_name, 'web', 'js'), glob('web/js/*.js')),
        (os.path.join('share', package_name, 'web', 'css'), glob('web/css/*.css')),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
    ],
    zip_safe=True,
    maintainer='a',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Web interface for controlling DUCO robot arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_arm_web_server = robot_arm_web.web_server_node:main',
        ],
    },
)
