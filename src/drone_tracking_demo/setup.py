from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'drone_tracking_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ADD THESE TWO LINES
        ('share/drone_tracking_demo/urdf', ['urdf/simple_quadrotor.urdf']),
        ('share/drone_tracking_demo/launch', [
            'launch/display_quadrotor.launch.py',
            'launch/full_demo.launch.py',
        ]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dchen',
    maintainer_email='dschen2018@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'drone_simulator = drone_tracking_demo.drone_simulator:main',
		'path_tracker = drone_tracking_demo.path_tracker:main',
                'mpc_tracker = drone_tracking_demo.mpc_tracker:main',
        ],
    },
)
