from setuptools import setup

package_name = 'vision_bot/src'

setup(
    name=package_name,
    version='0.0.0',
    packages=['perception'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Vision bot with YOLO',
    license='TODO'
)