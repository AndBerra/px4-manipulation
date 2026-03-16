import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share         = get_package_share_directory('px4_manipulation')
    px4_offboard_share = get_package_share_directory('px4_offboard')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'px4_manipulation.yaml'),
        description='Full path to config yaml inside the package config/ folder'
    )

    waypoints_path_arg = DeclareLaunchArgument(
        'waypoints_path',
        description='Absolute path to waypoints.json in the source tree'
    )

    # PX4 visualizer — launches RViz + drone pose visualization
    px4_visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(px4_offboard_share, 'visualize.launch.py')
        ),
        launch_arguments={
            'rviz_config': os.path.join(pkg_share, 'config', 'waypoint_following.rviz'),
        }.items()
    )

    # PX4 manipulation node in waypoint following mode
    px4_manipulation = Node(
        package='px4_manipulation',
        executable='talker',
        name='px4_manipulation',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'follow_waypoints': True,
                'waypoints_path':   LaunchConfiguration('waypoints_path'),
            }
        ],
    )

    # Waypoint list visualizer — publishes markers to RViz
    waypoint_visualizer = Node(
        package='px4_manipulation',
        executable='rviz_waypoint_list_visualizer.py',
        name='waypoint_list_visualizer',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'waypoints_path': LaunchConfiguration('waypoints_path'),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        waypoints_path_arg,
        px4_visualizer,
        px4_manipulation,
        waypoint_visualizer,
    ])