from glob import glob
from setuptools import setup

package_name = 'base_athome_sim'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UruBots',
    maintainer_email='paezemo@gmail.com',
    description='Simulacion sencilla de Base@Home',
    license='Apache-2.0',
    entry_points={'console_scripts': ['sim_robot = base_athome_sim.sim_robot:main']},
)
