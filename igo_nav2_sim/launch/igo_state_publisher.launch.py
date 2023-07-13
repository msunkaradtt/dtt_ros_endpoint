import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import EnvironmentVariable, SetEnvironmentVariable
import launch_ros
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='igo_nav2_sim').find('igo_nav2_sim')
    pkg_world_share = launch_ros.substitutions.FindPackageShare(package='igo_gazebo_sim').find('igo_gazebo_sim')
    default_model_path = os.path.join(pkg_share, 'description/igo_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/ic_urdf_config.rviz')

    
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
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        laser_filter_node,
        # rviz_node
    ])
