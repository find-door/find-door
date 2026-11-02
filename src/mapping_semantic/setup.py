from glob import glob

from setuptools import find_packages, setup

package_name = 'mapping_semantic'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    package_data={'mapping_semantic.third_party': ['LICENSE', 'NOTICE']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='정원준',
    maintainer_email='prophet000228@gmail.com',
    description='합성 색상 fixture의 RGB-D 투영·정적 객체 지도 검증',
    license='TBD',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'semantic_mapper = mapping_semantic.node:main',
        'door_mapper = mapping_semantic.door_node:main',
        'nanodet_detect = mapping_semantic.nanodet_detector:main',
        'detector_node = mapping_semantic.detector_node:main',
    ]},
)
