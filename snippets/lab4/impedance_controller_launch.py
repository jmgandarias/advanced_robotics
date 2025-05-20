import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('uma_arm_control'),
        'config',
        'impedance_params.yaml'
    )

    impedance_controller_node = Node(
            package='uma_arm_control',
            executable='impedance_controller',
            name='impedance_controller',
            output='screen',
            parameters=[config]
        )

    return LaunchDescription([impedance_controller_node])