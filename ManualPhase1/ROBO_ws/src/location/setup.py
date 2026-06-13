import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'location'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Default per-object configs (read-only fallback; user overrides live in
        # ~/.roboarm/objects — see object_config.py).
        (os.path.join('share', package_name, 'objects'), glob('location/objects/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirat',
    maintainer_email='gurkiratsinghvirdi10@gmail.com',
    description='Location element: object-position sources, finder, to_target.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'camera = location.camera:main',
            'aruco_source = location.aruco_source:main',
            'color_source = location.color_source:main',
            'to_target = location.to_target:main',
            'finder = location.finder:main',
            'tune = location.tune:main',
        ],
    },
)
