from setuptools import setup, find_packages
import os
import glob

package_name = 'urdf_web_viewer'

def get_data_files():
    """Get all data files for third_party directory"""
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/urdf_web_viewer.launch.py']),
    ]
    
    # Add third_party directory files
    for root, dirs, files in os.walk('third_party'):
        for file in files:
            file_path = os.path.join(root, file)
            target_dir = os.path.join('lib/python3.10/site-packages', root)
            if target_dir not in [item[0] for item in data_files]:
                data_files.append((target_dir, []))
            # Find existing entry and add file
            for i, (dir_path, file_list) in enumerate(data_files):
                if dir_path == target_dir:
                    data_files[i] = (dir_path, file_list + [file_path])
                    break
    
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    description='ROS2 node that serves a web interface for URDF visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urdf_web_server = urdf_web_viewer.urdf_web_server:main',
        ],
    },
)
