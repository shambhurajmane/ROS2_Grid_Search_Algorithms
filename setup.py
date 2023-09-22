import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'motion_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shambhuraj',
    maintainer_email='shambhuraj@todo.todo',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid = motion_planning.grid_publisher:main',
            'plot = motion_planning.performance_plot:main',
            'bfs = motion_planning.bfs:main',
            'dfs = motion_planning.dfs:main',
            'dijkstra = motion_planning.dijkstra:main',
            'random = motion_planning.random:main',
        ],
    },

)
