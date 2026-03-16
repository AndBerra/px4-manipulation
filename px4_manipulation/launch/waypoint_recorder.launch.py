import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('px4_manipulation')

    # RViz config
    rviz_config = os.path.join(pkg_share, 'config', 'waypoint_recorder.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = '' 

    # wp file path save
    waypoints_path_arg = DeclareLaunchArgument(
        'waypoints_path',
        description='Absolute path to save waypoints.json in the source tree'
    )

    return LaunchDescription([
        waypoints_path_arg,
        # Waypoint recorder node
        Node(
            package='px4_manipulation',
            executable='rviz_waypoint_recorder.py',
            name='waypoint_recorder',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'waypoints_path': LaunchConfiguration('waypoints_path'),
            }],            
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config] if rviz_config else [],
        ),
    ])