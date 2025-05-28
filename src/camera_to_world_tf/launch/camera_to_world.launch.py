from platform import node
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

import ros2node

def generate_launch_description():
    
    package_name = 'camera_to_world_tf'
    urdf_path = os.path.join(
        get_package_share_directory(package_name), 'urdf', 'arm_with_camera.urdf.xml'
    )

    # Declare the launch argument for the URDF file
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    node_robot_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2"
    )

    node_camera_to_world = Node(
        package='camera_to_world_tf',
        executable='camera_to_world',
        name='camera_to_world',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_robot_state_gui)
    ld.add_action(node_rviz)
    ld.add_action(node_camera_to_world)

    return ld