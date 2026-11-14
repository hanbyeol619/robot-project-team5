from glob import glob

from setuptools import find_packages, setup

package_name = 'seedling_control'

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
    maintainer='jun',
    maintainer_email='sjjo0953@gmail.com',
    description='묘목심기 중앙관제 노드',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_node = seedling_control.control_node:main',
            'mock_robot_node = seedling_control.mock_robot_node:main',
            'tts_node = seedling_control.tts_node:main',
        ],
    },
)
