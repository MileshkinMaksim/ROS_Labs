from setuptools import find_packages, setup

package_name = 'robotl_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beb',
    maintainer_email='m.mileshki@g.nsu.ru',
    description='robotl',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotl_app = robotl_app.robotl_app:main',
            'robotl_app_with_camera = robotl_app.robot_app_with_camera:main',
        ],
    },
)