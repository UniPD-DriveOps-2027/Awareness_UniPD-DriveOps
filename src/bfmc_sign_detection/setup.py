from glob import glob
from setuptools import setup

package_name = 'bfmc_sign_detection'
setup(
    name=package_name,
    version='0.1.0',
    packages=['bfmc_sign_detection'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['road_sign_detector = bfmc_sign_detection.road_sign_detector:main']},
)
