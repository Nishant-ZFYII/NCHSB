from setuptools import setup, find_packages
import os

package_name = 'rc_nav_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # inside setup(..., data_files=[ ... ])
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rc_nav_bridge']),
        ('share/rc_nav_bridge', ['package.xml']),
        ('share/rc_nav_bridge/launch', ['launch/bridge_only.launch.py']),
        ('share/rc_nav_bridge/config', ['config/stamper.params.yaml']),
    ]


    ,
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Twist → TwistStamped bridge and launch glue for Ackermann controller + Nav2.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stamper = rc_nav_bridge.stamper:main',
            'world_odom_aligner = rc_nav_bridge.world_odom_aligner:main',
        ],
    },



)
