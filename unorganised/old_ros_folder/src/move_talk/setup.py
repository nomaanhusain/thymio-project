from setuptools import find_packages, setup

package_name = 'move_talk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/move_talk_launch.py']),
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
            'robot_move = move_talk.random_move:main',
            'publish_temperature = move_talk.publish_temperature:main',
            'subscribe_temperature = move_talk.subscribe_temperature:main',
            'subscribe_quad = move_talk.subscribe_quadrant:main',
        ],
    },
)
