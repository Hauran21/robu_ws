from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    node_distance_sub = Node(
        package="robu",
        executable="obstacle_avoidance",
        output="screen",
        # remappings=
        # parameters=
        # arguments=
    )
    
    exec_distance_pub = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', "'source /home/robot/work/robu_bhme21_ws/install/setup.bash && ros2 run robu distance_sensor'"],

        shell=True
    )
        
    ld = LaunchDescription()
    ld.add_action(node_distance_sub)
    ld.add_action(exec_distance_pub)
    
    return ld
