from setuptools import setup
import os
from glob import glob

package_name = 'ai_odom_3d_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name,'config'), glob(os.path.join('config','*'))),
        (os.path.join('share', package_name,'models'), glob(os.path.join('models','*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='author1, author2',
    author_email='your.email@example.com, your.email@example.com',
    maintainer='maintainer2',
    maintainer_email='your.email@example.com',
    description='Ai-aided legged robot localization - ROS2 package',
    license='BSD-2.0',
    entry_points={
        'console_scripts': [
        'tracking = ai_odom_3d_ros2.main:main',
        ],
    },
)
