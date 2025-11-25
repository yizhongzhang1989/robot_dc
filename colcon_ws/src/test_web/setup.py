from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'templates'), glob('test_web/templates/*.html')),
    ],
    install_requires=[
        'setuptools',
        'flask',
    ],
    zip_safe=True,
    maintainer='Yizhong Zhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Simple test web service using Flask',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_web_node = test_web.test_web_node:main',
        ],
    },
)
