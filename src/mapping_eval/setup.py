from setuptools import find_packages, setup

package_name = 'mapping_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='정원준',
    maintainer_email='prophet000228@gmail.com',
    description='매핑 입력 인터페이스와 실행 상태 검증 도구',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['check_interfaces = mapping_eval.check_interfaces:main'],
    },
)
