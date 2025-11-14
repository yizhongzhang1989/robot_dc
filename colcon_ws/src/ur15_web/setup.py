from setuptools import setup
import os
from glob import glob

package_name = 'ur15_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'web'), ['web/index.html']),
        (os.path.join('share', package_name, 'web', 'js'), glob(os.path.join('web', 'js', '*.js'))),
        (os.path.join('lib', package_name), [])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yizhongzhang1989',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='UR15 Web Interface Node for camera validation and control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur15_web_node = ur15_web.ur15_web_node:main',
        ],
    },
)