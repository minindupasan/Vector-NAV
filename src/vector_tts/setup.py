from setuptools import find_packages, setup

package_name = 'vector_tts'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VECTOR NAV Team',
    maintainer_email='todo@example.com',
    description='Kokoro TTS ROS2 node for VECTOR NAV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_node = vector_tts.tts_node:main',
        ],
    },
)
