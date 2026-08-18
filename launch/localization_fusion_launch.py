"""
Launch AMCL + the visual odometry node + the robot_localization EKF
that fuses them into one continuous /odometry/filtered stream.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('custom_amr_platform')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        )
    )

    visual_odometry_node = Node(
        package='custom_amr_platform',
        executable='visual_odometry_node',
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf_params.yaml')],
    )

    return LaunchDescription([amcl_launch, visual_odometry_node, ekf_node])
