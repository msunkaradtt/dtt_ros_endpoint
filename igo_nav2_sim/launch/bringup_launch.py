# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    local_launch_dir = os.path.join(get_package_share_directory('igo_nav2_sim'), 'launch')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    # alphasense_launch_dir = os.path.join(get_package_share_directory('alphasense'), 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    localization_method = LaunchConfiguration('localization_method')


    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    lifecycle_nodes = ['map_server']

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'localization_method',
            default_value='0_map_frame',
            description='Loclization method to be used. Settings are: nav2_stack (default), alphasense, 0_map_frame'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'map',
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        LogInfo(msg='---- Launch File ---------------------------'),
        LogInfo(msg='bringup_launch.py'),
        LogInfo(msg='---- Parameter settings --------------------'),
        LogInfo(msg=['Localization method set to: ', localization_method]),
        LogInfo(msg=['Using map from: ', map_yaml_file]),
        LogInfo(msg=['Using para file: ', params_file]),
        LogInfo(msg=['use_sim_time set to: ', use_sim_time]),
        LogInfo(msg='--------------------------------------------'),

        # Specify the actions
        # TODO Use an action subgroup / conditional node with the static_transform_publisher
        # TODO For alphasense start the alphasense driver
        # TODO Start state publisher with robot driver
        Node(
            condition=IfCondition(PythonExpression(["'", localization_method, "' == '0_map_frame'"])),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
            ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([alphasense_launch_dir, '/alphasense_localization_driver.launch.py']),
        #     condition=IfCondition(PythonExpression(["'", localization_method, "' == 'alphasense'"]))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
                                      
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    ])



