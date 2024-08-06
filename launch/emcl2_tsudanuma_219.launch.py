# SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution


def generate_launch_description():    
    tsudanuma_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            TextSubstitution(text=os.path.join(
                    get_package_share_directory('emcl2'), 
                    'launch', 'emcl2.launch.py'))]),
        launch_arguments={
            'map': [
                TextSubstitution(text=os.path.join(
                        get_package_share_directory('emcl2'), 
                        'config', 'map', 'map_tsudanuma_219.yaml'))],
            # 'map_topic_name': [
            #     TextSubstitution(text='map/localization')
            # ]
        }.items()
    )
    ld = LaunchDescription()
    ld.add_action(tsudanuma_launch)
    
    return ld
