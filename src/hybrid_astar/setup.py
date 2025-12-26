from setuptools import setup
from glob import glob
import os

package_name = 'hybrid_astar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='aminebensaid66@gmail.com',
    description='Hybrid A* pathfinding package for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hybrid_astar_node = hybrid_astar.hybrid_astar_node:main',
            'json_to_ros = hybrid_astar.json_to_ros:main',
            'yaml_to_ros = hybrid_astar.yaml_to_ros:main',
            'generate_path = hybrid_astar.generate_path:main',
            'web_handler = hybrid_astar.web_handler:main',
        ],
    },
)
