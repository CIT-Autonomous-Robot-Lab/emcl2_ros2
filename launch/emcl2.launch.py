import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    emcl2_dir = get_package_share_directory('emcl2')
    params_file = os.path.join(
        emcl2_dir, 'config', 'emcl2.param.yaml')

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_a = LaunchConfiguration('initial_pose_a')

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Estimate initial values in the x-coordinate axis of the robot')
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Estimate initial values in the y-coordinate axis of the robot')
    declare_initial_pose_a = DeclareLaunchArgument(
        'initial_pose_a',
        default_value='0.0',
        description='Estimate initial values in the yaw rotation axis of the robot')

    param_substitutions = {
        'initial_pose_x': initial_pose_x,
        'initial_pose_y': initial_pose_y,
        'initial_pose_a': initial_pose_a}

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    lifecycle_nodes = ['map_server']

    launch_node = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{'yaml_filename': map_yaml_file}],
                output='screen'),
            Node(
                name='emcl2',
                package='emcl2',
                executable='emcl2_node',
                parameters=[configured_params],
                output='screen'),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}])

        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_initial_pose_x)
    ld.add_action(declare_initial_pose_y)
    ld.add_action(declare_initial_pose_a)

    ld.add_action(launch_node)

    return ld
