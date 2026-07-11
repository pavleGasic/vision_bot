from setuptools import find_packages, setup

package_name = 'visionbot_perception'

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
    maintainer='gasa14',
    maintainer_email='pavle.gasic7@gmail.com',
    description='Main ROS2 package for VisionBot',
    license='GPL-2.0 licence',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
