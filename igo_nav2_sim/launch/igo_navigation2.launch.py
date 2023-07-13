# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_method = LaunchConfiguration('localization_method')
    igo_prefix = get_package_share_directory('igo_nav2_sim')
    igo_sim_prefix = get_package_share_directory('igo_gazebo_sim')
    # Use warheouse example
    if False:
        if False:
            # Simple warehouse
            map_dir = LaunchConfiguration(
                'map',
                default=os.path.join(
                    igo_prefix,
                    'maps',
                    'sim_warehouse.yaml'))
        else:
            # Workshop warehouse
            map_dir = LaunchConfiguration(
                'map',
                default=os.path.join(
                    igo_sim_prefix,
                    'maps',
                    'sim_workshop_warehouse.yaml'))
    # Use vehicle
    else:
        map_dir = LaunchConfiguration(
            'map',
            default=os.path.join(
                igo_prefix,
                'maps',
                'real_warehouse.yaml'))

    param_file_name = 'igo_navigation.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            igo_prefix,
            'params',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    local_launch_dir = os.path.join(get_package_share_directory('igo_nav2_sim'), 'launch')

    rviz_config_dir = os.path.join(
        igo_prefix,
        'rviz',
        # 'igo_nav2_view.rviz')
        'igo_nav2_view_red.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization_method',
            default_value='nav2_stack',
            description='Loclization method to be used. Settings are: nav2_stack (default), alphasense, 0_map_frame'),

        # Depending on the selected localization_method either the nav2-stack-bringup is used or the 
        # project specific bringup setting up the localization as specified by localization_method
        LogInfo(msg='------------ Parameter settings ------------'),
        LogInfo(msg='ic_bot_navigation2_launch.py'),
        LogInfo(msg='----------------------------'),
        LogInfo(msg=['Localization method set to: ', localization_method]),
        LogInfo(msg=['Using map from: ', map_dir]),
        LogInfo(msg=['Using para file: ', param_dir]),
        LogInfo(msg=['use_sim_time set to: ', use_sim_time]),
        LogInfo(msg='------------ Parameter settings ------------'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([local_launch_dir, '/nav2_bringup_launch.py']),
            condition=IfCondition(PythonExpression(["'", localization_method, "' == 'nav2_stack'"])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([local_launch_dir, '/bringup_launch.py']),
            condition=UnlessCondition(PythonExpression(["'", localization_method, "' == 'nav2_stack'"])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'localization_method': localization_method}.items(),
            ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
