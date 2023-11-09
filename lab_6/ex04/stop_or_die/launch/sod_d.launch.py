import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_bringup'), 'launch'),
                                       '/4_box.launch.py']),
    )


    return LaunchDescription([
        robot_nodes,
        Node(
            package='stop_or_die',
            executable='sod_d',
            name='soad',
        ),
    ])
