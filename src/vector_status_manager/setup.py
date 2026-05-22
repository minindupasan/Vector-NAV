from glob import glob

from setuptools import find_packages, setup

package_name = 'vector_status_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='minindupasan@gmail.com',
    description='Unified robot status aggregator',
    license='MIT',
    entry_points={
        'console_scripts': [
            'status_manager_node = vector_status_manager.status_manager_node:main',
            'jetson_stats_node = vector_status_manager.jetson_stats_node:main',
        ],
    },
)
