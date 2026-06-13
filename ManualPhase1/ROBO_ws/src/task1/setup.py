import os
from glob import glob
from setuptools import setup

package_name = 'task1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirat',
    maintainer_email='kirat@todo.todo',
    description='visual servo control + task 1 tomato pick',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = task1.object_detector:main',
            'visual_servo = task1.visual_servo_controller:main',
            'moveit_interface = task1.moveit_interface:main',
            'joint_tracker = task1.joint_tracker:main',
            'tomato_detector = task1.tomato_detector:main',
            'pick_tomato = task1.pick_tomato:main',
            'estop = task1.estop:main',
        ],
    },
)
