from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'web'), ['web/index.html', 'web/tailwind.js']),
        (os.path.join('share', package_name, 'web', 'js'), glob(os.path.join('web', 'js', '*.js'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yizhongzhang1989',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Task Manager Web Interface for robot task management and monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = task_manager.task_manager_node:main',
        ],
    },
)
