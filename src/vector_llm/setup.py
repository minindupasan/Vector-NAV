from setuptools import find_packages, setup

package_name = 'vector_llm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vector_chatbot.launch.py']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='VECTOR NAV Team',
    maintainer_email='todo@example.com',
    description='LLM communication node for VECTOR NAV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_node = vector_llm.llm_node:main',
        ],
    },
)
