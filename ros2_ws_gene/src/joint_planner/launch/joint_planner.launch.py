from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
            package='joint_planner',
            executable='joint_planner',
            name='joint_planner',
            output='screen',
            # parameters=[{'use_sim_time': False}]
        )

    return LaunchDescription([
        node
    ])