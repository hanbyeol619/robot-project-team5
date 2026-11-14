from glob import glob

from setuptools import find_packages, setup

package_name = 'seedling_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team5',
    maintainer_email='team5@todo.todo',
    description='묘목심기 관제 UI (웹페이지 + 작은 웹서버)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ui_server = seedling_ui.ui_server:main',
        ],
    },
)
