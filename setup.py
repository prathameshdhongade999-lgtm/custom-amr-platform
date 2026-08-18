import os
from glob import glob
from setuptools import setup

package_name = 'custom_amr_platform'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prathamesh Dhongade',
    maintainer_email='prathameshdhongade999@gmail.com',
    description='AMR navigation stack fusing AMCL and visual odometry for a stable 15Hz control loop.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_odometry_node = custom_amr_platform.visual_odometry_node:main',
            'control_loop_node = custom_amr_platform.control_loop_node:main',
        ],
    },
)
