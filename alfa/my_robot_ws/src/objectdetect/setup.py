from setuptools import setup
import os
from glob import glob

package_name = 'objectdetect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # Sadece klasör adını yazıyoruz
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fatih',
    description='Mistik Nesne Ayıklama',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'objectdetect = objectdetect.pcl:main'
        ],
    },
)
