from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('px4_manipulation')
    px4_offboard_share = get_package_share_directory('px4_offboard')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'px4_manipulation.yaml'),
        description='Path to px4_manipulation config yaml inside the package config/ folder'
    )

    config_path = PathJoinSubstitution([
        FindPackageShare('px4_manipulation'),
        'config',
        LaunchConfiguration('config_file')
    ])
 
    manipulation_node = Node(
        package='px4_manipulation',
        namespace='px4_manipulation',
        executable='talker',
        name='sim',
        parameters=[config_path]
    )    
    targetpose_marker = Node(
        package='px4_manipulation',
        namespace='px4_manipulation',
        executable='rviz_targetpose_marker.py',
        name='targetpose_marker'
    )
    px4_visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(px4_offboard_share, 'visualize.launch.py')
        ),
        launch_arguments={
            'rviz_config': os.path.join(pkg_share, 'config', 'run.rviz'),
        }.items()
    )

    return LaunchDescription([
        config_file_arg,
        manipulation_node,
        px4_visualizer,
        targetpose_marker
])
