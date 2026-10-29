import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_bringup_jetson'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCup @Home Team',
    maintainer_email='athome@robot.org',
    description='Jetson Orin Nano Bringup (Vision, Pan-Tilt, Audio)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
