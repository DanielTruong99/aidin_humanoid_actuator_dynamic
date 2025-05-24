from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
            package='motor_driver',
            executable='combine_test',
            name='motor_driver',
            output='screen',
            # parameters=[{'use_sim_time': False}]
        )

    return LaunchDescription([
        node
    ])