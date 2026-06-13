import os
from glob import glob
from setuptools import setup

package_name = 'tasks'

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
    maintainer_email='gurkiratsinghvirdi10@gmail.com',
    description='Task element: object-agnostic high-level task orchestrators '
                '(pick) plus the global e-stop helper.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick = tasks.pick:main',
            'estop = tasks.estop:main',
        ],
    },
)
