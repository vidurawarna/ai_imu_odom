from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dog_imu_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name,'config'), glob(os.path.join('config','*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vidura',
    maintainer_email='vidurawarna99@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        'odom_to_imu_tf_broadcaster = dog_imu_test.odom_to_imu_tf_broadcaster:main',
        'flip_odom = dog_imu_test.flip_odom:main',
        'lidar_odom_repub = dog_imu_test.lidar_odom_repub:main',
        'ekf_node = dog_imu_test.ekf_node:main',
        'ekf_pos_error_node = dog_imu_test.ekf_pos_error:main',
        ],
    },
)
