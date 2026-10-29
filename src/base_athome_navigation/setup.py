import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'base_athome_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'config', 'bt_trees'), glob('config/bt_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='athome',
    maintainer_email='athome@base.local',
    description='Navegación Nav2 para Base@Home',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = base_athome_navigation.waypoint_navigator:main',
            'map_zone_editor = base_athome_navigation.map_zone_editor:main',
        ],
    },
)
