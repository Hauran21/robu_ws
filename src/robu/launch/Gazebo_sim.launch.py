import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro
import subprocess
import re


#Arbeitsschritte in der CLI:
#############################################################

#SDF ist kein URDF!!! Umwandlung von URDF in SDF notwendig
# 1.) xacro  src/robu/urdf/example_robot.urdf.xacro > src/robu/urdf/example_robot.urdf
# 2.) gz sdf -p src/robu/urdf/example_robot.urdf > src/robu/urdf/example_robot.sdf
# 3.) export GAZEBO_MODEL_PATH=/home/robu/work/ROBU/robu_bhme21_ws/src/robu/models:$GAZEBO_MODEL_PATH

#Gazebo starten
# Hinweis: Gazebo Server mit verbose Flag starten ->  leichter für die Fehlersuche -> es werden logging Informationen ausgegeben!
# Mit eigener Welt starten (die Welt muss einen Boden haben sonst fällt der Roboter in Nirvana!)
# 4.) gazebo --verbose /home/robu/work/ROBU/robu_bhme21_ws/src/robu/worlds/test.world
# oder mit einer leeren Welt starten (Standardwelt ist empty.world -> /usr/share/gazebo-11/worlds/empty.world):
# 4.) gazebo --verbose
#Roboter in Gazebo spawnen 
# 5.) gz model --spawn-file /home/robu/work/ROBU/robu_bhme21_ws/src/robu/urdf/example_robot.sdf --model-name example_robot

#Vergleiche mit spawn_turtlebot3.launch.py:
    # start_gazebo_ros_spawner_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', TURTLEBOT3_MODEL,
    #         '-file', urdf_path,    -> Unterschied -topic robot_description und -file urdf_path!
    #         '-x', x_pose,
    #         '-y', y_pose,
    #         '-z', '0.01'
    #     ],
    #     output='screen',
#https://answers.gazebosim.org/question/24214/how-to-spawn-a-robot-in-gazebo-using-sdf-file/


def urdf_to_sdf(urdf_file:str, sdf_file:str):

    temporary_urdf_file = os.path.splitext(urdf_file)[0] + ".tmp.urdf"

    os.system(f"xacro {urdf_file} > {temporary_urdf_file}")
    with open(temporary_urdf_file, 'r') as file:
        robot_description = file.read()
    
    robot_description = re.sub(r'filename="package://[^/]+/[^/]+/', 'filename="model://', robot_description)
    with open(temporary_urdf_file, 'w') as file:
        file.write(robot_description)

    os.system(f"gz sdf -p {temporary_urdf_file} > {sdf_file}")

    os.remove(temporary_urdf_file)


def generate_launch_description():
    pkg_name = 'robu'
    robot_name = 'example_robot'

    robot_urdf_filename = 'example_robot.urdf.xacro'
    robu_shared_dir = get_package_share_directory(pkg_name)
    robot_urdf_filepath = os.path.join(robu_shared_dir, 'urdf', robot_urdf_filename)

    robot_description = xacro.process_file(robot_urdf_filepath).toxml()

    robot_sdf_file = os.path.join(robu_shared_dir, 'urdf', os.path.splitext(robot_urdf_filename)[0] + ".sdf")
    urdf_to_sdf(robot_urdf_filepath, robot_sdf_file)
    
    larg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true')
    larg_worldfile = DeclareLaunchArgument('world', default_value='')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo_model_path = os.path.join(get_package_share_directory(pkg_name), "urdf/")
    envar_gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        gazebo_model_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")
    )
    envar_gazebo_model_database_uri = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    node_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        #name="robot_state_publisher",
        output='screen',
        parameters=[{'robot_description': robot_description,
        'use_sim_time': use_sim_time}],
    )

    node_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-file', robot_sdf_file,
                                '-entity', robot_name],
                    output='screen')
    
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2"
    )
 
    ld = LaunchDescription()

    ld.add_action(larg_use_sim_time)
    ld.add_action(larg_worldfile)
    ld.add_action(envar_gazebo_model_path)
    ld.add_action(node_state_publisher)
    ld.add_action(node_state_gui)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(node_rviz)
    

    return ld

#    gazebo --verbose /usr/share/gazebo-11/worlds/empty.world
#    gz model --spawn-file /home/robu/work/robu_bhme21_ws/src/robu/urdf/example_robot.sdf --model-name example_robot