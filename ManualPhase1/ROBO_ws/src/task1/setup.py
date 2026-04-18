from setuptools import setup

package_name = 'task1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirat',
    maintainer_email='kirat@todo.todo',
    description='visual servo control',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = task1.object_detector:main',
            'visual_servo = task1.visual_servo_controller:main',
            'moveit_interface = task1.moveit_interface:main',
            'joint_tracker = task1.joint_tracker:main',
        ],
    },
)
