from setuptools import find_packages, setup
from glob import glob

package_name = 'seedling_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/resource', ['resource/.env']),   # 이 줄 추가
    ('share/' + package_name + '/resource', glob('resource/*.onnx') + glob('resource/*.tflite')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='sjjo0953@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_node = seedling_llm.llm_node:main',
            'voice_node = seedling_llm.voice_node:main',
            'voice_llm_node = seedling_llm.voice_llm_node:main',
        ],
    },
)
