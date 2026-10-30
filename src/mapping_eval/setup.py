from glob import glob

from setuptools import find_packages, setup

package_name = 'mapping_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='정원준',
    maintainer_email='prophet000228@gmail.com',
    description='매핑 입력 인터페이스와 실행 상태 검증 도구',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_interfaces = mapping_eval.check_interfaces:main',
            'evaluate_map = mapping_eval.evaluate_map:main',
            'drive_route = mapping_eval.drive_route:main',
            'analyze_trajectory = mapping_eval.trajectory_metrics:main',
            'analyze_graph = mapping_eval.graph_motion:main',
            'run_semantic_benchmark = mapping_eval.run_semantic_benchmark:main',
            'replay_benchmark = mapping_eval.replay_benchmark:main',
            'capture_diagnostics = mapping_eval.diagnostics_node:main',
            'audit_startup = mapping_eval.startup_audit:main',
            'run_benchmark = mapping_eval.run_benchmark:main',
        ],
    },
)
