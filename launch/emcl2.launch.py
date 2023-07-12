import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():
    emcl2_dir = get_package_share_directory('emcl2')
    params_file = os.path.join(
        emcl2_dir, 'config', 'emcl2.param.yaml')

    launch_node = GroupAction([
        Node(
            name='emcl2',
            package='emcl2',
            executable='emcl2_node',
            parameters=[params_file],
            output='screen')
    ])

    ld = LaunchDescription()
    ld.add_action(launch_node)

    return ld
