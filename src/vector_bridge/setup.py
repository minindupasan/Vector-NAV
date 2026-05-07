from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vector_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('lib', package_name, 'static'),
            glob('vector_bridge/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vector Nav',
    maintainer_email='minindupasan@gmail.com',
    description='Web control dashboard bridge for Vector Nav',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = vector_bridge.bridge_node:main',
        ],
    },
)
