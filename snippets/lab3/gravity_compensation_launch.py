import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('uma_arm_control'),
        'config',
        'dynamics_params.yaml'
    )

    gravity_compensation_node = Node(
            package='uma_arm_control',
            executable='gravity_compensation',
            name='gravity_compensation',
            output='screen',
            parameters=[config]
        )

    return LaunchDescription([gravity_compensation_node])