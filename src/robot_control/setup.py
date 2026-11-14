from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bfcreator',
    maintainer_email='bfcreator@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_basic = robot_control.move_basic:main',
            'mini_jog_ = robot_control.mini_jog_:main',
            'move_grip = robot_control.move_grip:main',
            'seedling_robot = robot_control.seedling_robot:main',
        ],
    },
)
