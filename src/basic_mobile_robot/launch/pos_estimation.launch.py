from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basic_mobile_robot',
            executable='pos_estimation.py',
            name='pos_estimation',
            output='screen'
        ),
    ])

