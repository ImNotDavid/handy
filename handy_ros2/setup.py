from setuptools import find_packages, setup

package_name = 'handy_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'talker = handy_ros2.publisher_member_function:main',
        ],
    },
)
