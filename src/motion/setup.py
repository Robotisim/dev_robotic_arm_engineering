import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='motion Maintainer',
    maintainer_email='maintainer@example.com',
    description='Numbered ROS 2 trajectory + IK lecture nodes for Panda arm motion demos.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_01_point_to_point = motion.demo_01_point_to_point:main',
            'motion_02_straight_line = motion.demo_02_straight_line:main',
            'motion_03_circle_bump = motion.demo_03_circle_bump:main',
            'motion_04_rectangle_shape = motion.demo_04_rectangle_shape:main',
            'motion_05_workspace_reject = motion.demo_05_workspace_reject:main',
            'motion_06_moveit_pose_goal = motion.demo_06_moveit_pose_goal:main',
        ],
    },
)
