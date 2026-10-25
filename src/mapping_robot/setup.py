from glob import glob
from setuptools import setup

setup(name='mapping_robot', version='0.1.0', packages=['mapping_robot'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/mapping_robot']),
                  ('share/mapping_robot', ['package.xml', 'README.md']),
                  ('share/mapping_robot/launch', glob('launch/*.launch.py')),
                  ('share/mapping_robot/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='정원준', maintainer_email='prophet000228@gmail.com',
      description='실물 프로파일·센서 검사·SLAM 실행', license='TBD',
      tests_require=['pytest'],
      entry_points={'console_scripts': [
          'validate_profile = mapping_robot.profile:main',
          'preflight = mapping_robot.preflight:main']})
