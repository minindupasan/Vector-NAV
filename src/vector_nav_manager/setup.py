from setuptools import find_packages, setup

package_name = 'vector_nav_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='minindupasan@gmail.com',
    description='Waypoint navigation manager',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nav_manager = vector_nav_manager.nav_manager_node:main',
        ],
    },
)
