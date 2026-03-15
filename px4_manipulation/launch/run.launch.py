from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('px4_manipulation'),
        'config',
        'px4_manipulation.yaml'
    )

    manipulation_node = Node(
        package='px4_manipulation',
        namespace='px4_manipulation',
        executable='talker',
        name='sim',
        parameters=[params_file]
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
        manipulation_node,
        px4_visualizer,
        targetpose_marker
])
