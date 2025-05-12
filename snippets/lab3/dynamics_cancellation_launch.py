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

    dynamics_cancellation_node = Node(
            package='uma_arm_control',
            executable='dynamics_cancellation',
            name='dynamics_cancellation',
            output='screen',
            parameters=[config]
        )

    return LaunchDescription([dynamics_cancellation_node])