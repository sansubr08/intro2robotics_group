import os
from glob import glob
from setuptools import setup

package_name = 'gazebo_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'diff_robot'),
            glob('models/diff_robot/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='you@example.com',
    description='Gazebo diff-drive robot with PD go-to-goal controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pid_controller = gazebo_controller.pid_controller:main',
        ],
    },
)
