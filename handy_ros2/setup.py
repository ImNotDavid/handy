from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'handy_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'urdf/assets'), glob(os.path.join('urdf/assets', '**'))),
        (os.path.join('share', package_name, package_name), glob(f'{package_name}/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='winton',
    maintainer_email='dc1021@ic.ac.uk',
    description='Publish joints from ESP32',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = handy_ros2.handy_bridge:main',
            'forward = handy_ros2.handy_forward:main'
        ],
    },
)
