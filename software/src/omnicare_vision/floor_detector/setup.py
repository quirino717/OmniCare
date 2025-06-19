from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'floor_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', glob('weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thiago M.',
    maintainer_email='rpj134@gmail.com',
    description='Package for floor detection inside the elevator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'floor_detector_node = floor_detector.floor_detector_node:main',
        ],
    },
)
