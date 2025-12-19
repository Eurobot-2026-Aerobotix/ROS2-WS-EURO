from setuptools import setup

package_name = 'hybrid_astar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
