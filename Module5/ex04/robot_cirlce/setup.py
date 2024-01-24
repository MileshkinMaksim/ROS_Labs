from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_cirlce'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beb',
    maintainer_email='m.mileshki@g.nsu.ru',
    description='robot circle movement',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_movement = robot_cirlce.circle_movement:main',
            'circling = robot_cirlce.circling:main'
        ],
    },
)
