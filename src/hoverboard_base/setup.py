from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hoverboard_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'urdf', 'xacro'), glob('urdf/xacro/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ShiiSuii',
    maintainer_email='paezemo@gmail.com',
    description='ROS 2 hardware driver and base bringup for hoverboard differential mobile base',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hoverboard_driver = hoverboard_base.hoverboard_driver:main',
            'hoverboard_teleop_joy = hoverboard_base.teleop_joy:main',
        ],
    },
)
