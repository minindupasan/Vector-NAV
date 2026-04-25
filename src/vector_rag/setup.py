from setuptools import find_packages, setup

package_name = 'vector_rag'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # Install knowledge_base/ and data/ inside the Python package so that
    # Path(__file__).parent / 'knowledge_base' resolves correctly at runtime.
    package_data={
        package_name: [
            'knowledge_base/*.txt',
            'data/*',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VECTOR NAV Team',
    maintainer_email='todo@example.com',
    description='RAG engine ROS2 node for VECTOR NAV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rag_node = vector_rag.rag_node:main',
        ],
    },
)
