from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'algo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # algo + mapping otomatik bulunur
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # YOLO weights ve model dosyaları
        (os.path.join('share', package_name, 'mapping'), glob('mapping/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gauss',
    maintainer_email='gauss@todo.com',
    description='Waypoint generator + SLAM pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_generator = algo.waypoint_generator:main',
            'withmap = mapping.withmap:main',
        ],
    },
)
