from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vector_teleop'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@vector.nav',
    description='Beautiful Qt-based teleoperation GUI for VECTOR NAV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_gui = vector_teleop.teleop_gui:main',
        ],
    },
)
