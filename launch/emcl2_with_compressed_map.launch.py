import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('binary_image_compressor'), 'map', 'mile1_2_50.yaml'),
        description='Full path to map yaml file to load')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('emcl2'), 'config', '')),
            TextSubstitution(text='emcl2.param.yaml')],
        description='emcl2 param file path')

    # パッケージディレクトリを取得
    binary_image_compressor_dir = get_package_share_directory('binary_image_compressor')
    emcl2_dir = get_package_share_directory('emcl2')

    # パラメータファイルのパスを設定
    compressor_param_file = os.path.join(binary_image_compressor_dir, 'config', 'compressor_params.yaml')
    emcl2_param_file = os.path.join(emcl2_dir, 'config', 'emcl2.param.yaml')

    # パラメータファイルが存在するか確認
    if not os.path.exists(compressor_param_file):
        print(f"警告: 圧縮機のパラメータファイルが見つかりません: {compressor_param_file}")
    if not os.path.exists(emcl2_param_file):
        print(f"警告: EMCL2のパラメータファイルが見つかりません: {emcl2_param_file}")

    launch_node = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),

            # EMCL2ノード
            Node(
                name='emcl2',
                package='emcl2',
                executable='emcl2_node',
                parameters=[emcl2_param_file],
                output='screen'),

                        # 圧縮マップパブリッシャー
            Node(
                package='binary_image_compressor',
                executable='compressed_image_publisher',
                name='compressed_image_publisher',
                parameters=[compressor_param_file],
                output='screen',
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)

    ld.add_action(launch_node)

    return ld
