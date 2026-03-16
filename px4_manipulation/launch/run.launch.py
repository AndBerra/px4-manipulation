from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('px4_manipulation')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'px4_manipulation.yaml'),
        description='Path to px4_manipulation config yaml inside the package config/ folder'
    )

    manipulation_node = Node(
        package='px4_manipulation',
        namespace='px4_manipulation',
        executable='talker',
        name='sim',
        parameters=[LaunchConfiguration('config_file')]
    )    
    targetpose_marker = Node(
        package='px4_manipulation',
        namespace='px4_manipulation',
        executable='rviz_targetpose_marker.py',
        name='targetpose_marker'
    )
    px4_visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px4_offboard'),
                'visualize.launch.py'
            )
        )
    )
    return LaunchDescription([
        config_file_arg,
        manipulation_node,
        px4_visualizer,
        targetpose_marker
])
