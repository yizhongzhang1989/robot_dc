from setuptools import setup
from glob import glob
import os

package_name = 'ur15_workflow'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.handlers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'examples'), glob('examples/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot DC Team',
    maintainer_email='robot@example.com',
    description='UR15 Workflow Control Package - Modular workflow-driven robot operations',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_workflow = ur15_workflow.runner:main',
        ],
    },
)
