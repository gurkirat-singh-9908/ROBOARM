from setuptools import setup
import os
from glob import glob

package_name = 'handeye_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirat',
    maintainer_email='gurkiratsinghvirdi10@gmail.com',
    description='Hand-eye calibration for the RoboArm (cv2.calibrateHandEye).',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sample_collector = handeye_calibration.sample_collector:main',
            'compute_calibration = handeye_calibration.compute_calibration:main',
            'publish_calibration = handeye_calibration.publish_calibration:main',
        ],
    },
)
