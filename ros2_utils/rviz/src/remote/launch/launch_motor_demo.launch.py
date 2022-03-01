import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_manipulator = os.path.join(get_package_share_directory('micro-ros_motor_demo_robot-description'),
                    'urdf', 'manipulator.xml')

    rviz_config_manipulator = os.path.join(get_package_share_directory('micro-ros_motor_demo'),
                               'config', 'manipulator.rviz')

    urdf = open(urdf_manipulator).read()

    return LaunchDescription([
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}, {'ignore_timestamp': True}]),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_manipulator]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': urdf}, {'source_list': ['/motor/joint']}]),
        ])