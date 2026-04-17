from setuptools import find_packages, setup

package_name = 'ArUcoMarkerRos'

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
        'camera_node = ArUcoMarkerRos.camera_node:main',
        'aruco_detect = ArUcoMarkerRos.aruco_detect:main',
        ],
    },
)
