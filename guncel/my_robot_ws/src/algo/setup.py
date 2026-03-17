from setuptools import setup

package_name = 'algo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gauss',
    maintainer_email='gauss@todo.com',
    description='Waypoint generator algoritması',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_generator = algo.waypoint_generator:main',
        ],
    },
)
