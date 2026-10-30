from glob import glob

from setuptools import find_packages, setup

package_name = 'base_athome_skills'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='athome',
    maintainer_email='athome@base.local',
    description='Habilidades RoboCup@Home sin control de actuadores',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_open_detector = base_athome_skills.door_open_node:main',
        ],
    },
)
