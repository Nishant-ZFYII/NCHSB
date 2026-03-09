from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'corridor_social_nav_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scenarios'),
            glob('scenarios/*.yaml')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nishant',
    maintainer_email='nishant@nyu.edu',
    description='Experiment infrastructure for corridor social navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gt_people_publisher = corridor_social_nav_bringup.gt_people_publisher:main',
            'gt_collision_detector = corridor_social_nav_bringup.gt_collision_detector:main',
            'metrics_logger = corridor_social_nav_bringup.metrics_logger:main',
            'goal_sender = corridor_social_nav_bringup.goal_sender:main',
            'pedestrian_driver = corridor_social_nav_bringup.pedestrian_driver:main',
            'cmd_vel_logger = corridor_social_nav_bringup.cmd_vel_logger:main',
        ],
    },
)
