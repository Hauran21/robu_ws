from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    arg_mode = DeclareLaunchArgument('mode', default_value='1', description='1...Interantional, 2...Ohne Gelb-Phase, 3...Aus, 4...Baustellenmodus')

    node_traffic_lights = Node(
        package='traffic_lights',
        executable='traffic_lights',
        name='traffic_light_node',
        parameters=[{
            'mode': LaunchConfiguration('mode')
        }]
    )

    ld = LaunchDescription()
    ld.add_action(arg_mode)
    ld.add_action(node_traffic_lights)
    return ld