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
            'manipulacion_autonoma = scripts.manipulacion_autonoma:main',
            'manipulacion_autonoma.py = scripts.manipulacion_autonoma:main',
            'calibrar_aruco_manipulacion = scripts.calibrar_aruco_manipulacion:main',
            'calibrar_aruco_manipulacion.py = scripts.calibrar_aruco_manipulacion:main',
            'hri_receptionist = scripts.hri_receptionist_challenge:main',
            'hri_receptionist.py = scripts.hri_receptionist_challenge:main',
            'challenge_hri = scripts.hri_receptionist_challenge:main',
            'animated_face = scripts.animated_face_node:main',
            'animated_face.py = scripts.animated_face_node:main',
        ],
    },
)
