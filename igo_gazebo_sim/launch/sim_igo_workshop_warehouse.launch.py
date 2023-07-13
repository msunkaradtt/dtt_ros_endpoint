import launch
from launch.actions import LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
import launch_ros
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='igo_nav2_sim').find('igo_nav2_sim')
    pkg_world_share = launch_ros.substitutions.FindPackageShare(package='igo_gazebo_sim').find('igo_gazebo_sim')
    default_model_path = os.path.join(pkg_share, 'description/igo_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/ic_urdf_config.rviz')
    # Worlds including models of this package
    world_path=os.path.join(pkg_world_share, 'worlds/workshop_warehouse.world')

    # For the import of the Gazebo model
    set_gazebo_envvar = SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(get_package_share_directory('igo_gazebo_sim'), 'models'))
    log1_msg = LogInfo(msg=['Set env-var GAZEBO_MODEL_PATH = ', EnvironmentVariable('GAZEBO_MODEL_PATH')])
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    laser_filter_node = launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name='scan_to_scan_filter_chain',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('igo_nav2_sim'),
                "config", "igo_laser_filter.yaml",
            ])],
        remappings=[
            ('scan', 'scan_raw'),
            ('scan_filtered', 'scan')
        ]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'igo_neo', '-topic', 'robot_description', '-x', '-3.5', '-y', '-5.5', '-Y', '0.0'],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),

        # Handle paths
        set_gazebo_envvar,
        log1_msg,

        # For gui use 'gazebo', for headless use 'gzserver'
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        launch.actions.ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        laser_filter_node,
        spawn_entity,
        # rviz_node
    ])
