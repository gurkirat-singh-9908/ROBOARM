from setuptools import find_packages, setup

package_name = 'roboticarm_control'

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
    maintainer='kirat',
    maintainer_email='gurkiratsinghvirdi10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'move_robot = roboticarm_control.move_robot:main',
            'object_detector = roboticarm_control.object_detector:main',
            'object_follower = roboticarm_control.object_follower:main',
],
    },
)
