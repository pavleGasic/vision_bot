from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'visionbot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pavle Gasic',
    maintainer_email='pavle.gasic7@gmail.com',
    description='ROS 2 package for visual perception and object classification',
    license='GPL-2.0 licence',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'yolo_detector = visionbot_perception.yolo_detector:main'
        ],
    },
)
