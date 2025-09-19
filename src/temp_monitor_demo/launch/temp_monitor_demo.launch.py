from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    arg_threshold = DeclareLaunchArgument('threshold', 
                                            default_value='50.0', 
                                            description='Temperature threshold for alerts')

    arg_hz = DeclareLaunchArgument('hz', 
                                      default_value='1.0', 
                                      description='Sampling rate in Hz')    

    temp_monitor_node = Node(
        package='temp_monitor_demo',
        executable='temp_monitor',
        name='temp_monitor',
        parameters=[{
            'hz': LaunchConfiguration('hz'),
            #'mean': LaunchConfiguration('25.0'),
            #'amp': LaunchConfiguration('5.0'),
        }]
    )
    
    temp_sensor_node = Node(
        package='temp_monitor_demo',
        executable='temp_sensor',
        name='temp_sensor',
        parameters=[{
            'threshold': LaunchConfiguration('threshold')
        }]
    )

    ld = LaunchDescription()
    ld.add_action(arg_threshold)
    ld.add_action(arg_hz)
    
    ld.add_action(temp_monitor_node)
    ld.add_action(temp_sensor_node)
    return ld