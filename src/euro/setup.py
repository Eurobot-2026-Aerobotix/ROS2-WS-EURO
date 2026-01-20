from setuptools import find_packages, setup

package_name = 'euro'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/tasks.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='aminebensaid66@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_agents = euro.start_esp32_agents:main',
            'stm32_agent = euro.start_stm32_agent:main',
            'strategy_manager= euro.strategy_manager:main',
            'pami_controller = euro.pami_controller:main',
        ],
    },
)
