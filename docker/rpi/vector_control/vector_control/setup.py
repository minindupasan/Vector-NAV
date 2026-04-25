from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vector_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vector',
    maintainer_email='vector@vector.nav',
    description='Hardware control for VECTOR NAV — motor drivers, encoders, IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = vector_control.motor_driver_node:main',
            'imu_publisher = vector_control.imu_node:main',
        ],
    },
)
