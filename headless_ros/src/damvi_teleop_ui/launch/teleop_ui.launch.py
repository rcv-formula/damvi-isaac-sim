import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('damvi_teleop_ui'),
                          'config', 'teleop.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='damvi_teleop_ui', executable='teleop_ui',
             name='damvi_teleop_ui', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
