from glob import glob

from setuptools import find_packages, setup

package_name = 'ground_gimbal_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zyshen',
    maintainer_email='zyshen@seas.upenn.edu',
    description='Simulation model and controls for an AI-powered ground gimbal robot.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cinematic_tracking = ground_gimbal_robot.cinematic_tracking:main',
            'gimbal_demo = ground_gimbal_robot.gimbal_demo:main',
            'human_tracking = ground_gimbal_robot.human_tracking:main',
            'motion_demo = ground_gimbal_robot.motion_demo:main',
            'moving_target_demo = ground_gimbal_robot.moving_target_demo:main',
            'person_following = ground_gimbal_robot.person_following:main',
            'tracking_filter = ground_gimbal_robot.tracking_filter:main',
            'tracking_mode_director = ground_gimbal_robot.tracking_mode_director:main',
            'tracking_motion_demo = ground_gimbal_robot.tracking_motion_demo:main',
        ],
    },
)
