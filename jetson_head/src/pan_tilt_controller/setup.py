from setuptools import find_packages, setup

package_name = 'pan_tilt_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCup @Home Team',
    maintainer_email='athome@robot.org',
    description='ROBOTIS Dynamixel Pan-Tilt Controller for RoboCup @Home',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pan_tilt_node = pan_tilt_controller.pan_tilt_node:main',
        ],
    },
)
