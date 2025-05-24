from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = '/home/humanoid2/DanielTruong/aidin_humanoid_actuator_dynamic/isaac_lab_for_ik/source/isaac_lab_for_ik/isaac_lab_for_ik/assets/urdf/leg05/hr.urdf'  # or .xacro if preprocessed

    with open(urdf_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_content}]
        )
    ])
