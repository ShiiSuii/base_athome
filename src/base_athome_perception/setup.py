import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'base_athome_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='athome',
    maintainer_email='athome@base.local',
    description='Modulo de Percepcion y Vision Artificial para RoboCup @Home',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_recognition = scripts.test_face_recognition:main',
            'person_pose = scripts.test_human_pose:main',
            'garbage_detection = scripts.test_garbage_detection:main',
            'piper_teach_recorder = scripts.piper_teach_recorder:main',
            'pringles_grasping = scripts.pringles_grasping_node:main',
        ],
    },
)
