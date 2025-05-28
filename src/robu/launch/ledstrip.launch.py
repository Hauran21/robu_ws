from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    node_ledstrip_sub = Node(
        package="robu",
        executable="plf01_sub",
        output="screen",
        # remappings=
        # parameters=
        # arguments=
    )
    
    exec_ledstrip_pub = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', "'source /home/robot/work/robu_bhme21_ws/install/setup.bash && ros2 run robu plf01_pub'"],
        #cmd=['terminator', '-e', "'source /home/robot/work/robu_bhme21_ws/install/setup.bash && run robu plf01_pub'"],
        shell=True
        #cmd="ros2 run robu plf01_pub",
        #pty=True 
    )
    
    node_ledstrip_pub = Node(
        package="robu",
        executable="plf01_pub",
        output="screen",
        # remappings=
        # parameters=
        # arguments=
    )
    
    ld = LaunchDescription()
    ld.add_action(node_ledstrip_sub)
    ld.add_action(exec_ledstrip_pub)
    
    # ld.add_action(node_ledstrip_pub)
    
    return ld
