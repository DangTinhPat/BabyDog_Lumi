import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'joystick_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dvt',
    maintainer_email='dangtinh.ftcpy@gmail.com',
    description='Physical joystick (ros2 joy) and keyboard fallback -> /control_input',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_input = joystick_bridge.joystick_input:main',
            'keyboard_input = joystick_bridge.keyboard_input:main',
        ],
    },
)
