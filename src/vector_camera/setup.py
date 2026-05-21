from setuptools import find_packages, setup

package_name = 'vector_camera'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@vector.nav',
    description='IMX708 camera — standalone WebRTC stream (camera_stream.py)',
    license='MIT',
    entry_points={'console_scripts': []},
)
