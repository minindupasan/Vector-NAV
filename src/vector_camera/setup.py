from setuptools import find_packages, setup

package_name = 'vector_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@vector.nav',
    description='IMX708 camera driver and WebRTC streaming',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = vector_camera.camera_node:main',
            'webrtc_node = vector_camera.webrtc_node:main',
        ],
    },
)
