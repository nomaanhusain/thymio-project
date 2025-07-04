from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wind_direction_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thymio',
    maintainer_email='thymio@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_wind_direction = wind_direction_detector.publish_direction:main',
            'random_robot_move = wind_direction_detector.random_move:main',
        ],
    },
)
