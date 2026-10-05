import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoNav Team',
    maintainer_email='autonav@example.com',
    url='https://github.com/OrtizDiego/AutoNav_Sim',
    description='Differential drive robot simulation with autonomous navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_controller = my_bot.ball_controller:main',
            'ball_teleop = my_bot.ball_teleop:main',
            'ball_chaser = my_bot.ball_chaser:main',
            'sensor_fusion = my_bot.sensor_fusion:main',
            'person_controller = my_bot.person_controller:main',
            'person_tracker = my_bot.person_tracker:main',
            'security_guard_bt = my_bot.security_guard_bt:main',
            'system_monitor = my_bot.system_monitor:main',
            'perf_monitor = my_bot.perf_monitor:main',
            'target_tracker = my_bot.target_tracker:main',
        ],
    },
)