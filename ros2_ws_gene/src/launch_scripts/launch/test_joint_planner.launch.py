from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the other launch files
    joint_planner_pkg_share = get_package_share_directory('joint_planner')
    joint_planner_launch = os.path.join(joint_planner_pkg_share, 'launch', 'joint_planner.launch.py')

    return LaunchDescription([
        # Include the joint planner launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joint_planner_launch)
        ),
        
        # Optionally, add more nodes or actions here
    ])