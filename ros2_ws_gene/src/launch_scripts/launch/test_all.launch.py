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

    # Define the path to the motor driver launch file
    motor_driver_pkg_share = get_package_share_directory('motor_driver')
    motor_driver_launch = os.path.join(motor_driver_pkg_share, 'launch', 'motor_driver.launch.py')

    # Define the path to the ros2_xenomai_bridge launch file
    ros2_xenomai_bridge_pkg_share = get_package_share_directory('ros2_xenomai_bridge')
    ros2_xenomai_bridge_launch = os.path.join(ros2_xenomai_bridge_pkg_share, 'launch', 'ros2_xenomai_bridge.launch.py')

    return LaunchDescription([
        # Include the motor driver launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(motor_driver_launch)
        ),

        # Include the ros2_xenomai_bridge launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros2_xenomai_bridge_launch)
        ),

        # Include the joint planner launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joint_planner_launch)
        ),
        # Optionally, add more nodes or actions here
    ])