from math import sqrt, cos, sin
import rclpy
from turtlesim.msg import Pose
from rclpy.node import Node

'''
Starten vom Simulator:
ros2 run turtlesim turtlesim_node 

Datentyp abfragen:
ros2 topic info /turtle1/pose 

Serives anzeigen lassen:
ros2 service list

Spawnen einer zweiten Schildkröte:
ros2 service call /spawn turtlesim/srv/Spawn "x: 5.0
y: 10.0
theta: 10.0
name: 'turtle2'" 


Steuerung starten:
ros2 run turtlesim turtle_teleop_key 
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtle2/cmd_vel

Um nur mehr Warnungen auszugeben:
--ros-args --log-level WARN

'''

class SimpleKinematics(Node):

    def __init__(self, node_name:str):
        super().__init__(node_name) # super() => Greift auf die Überklasse auf "Node"
        # "self._sub_turtle1_pose_cb" = Callback => Callback wird aufgerufen wenn etwas empfangen wird
        self.create_subscription(Pose, "/turtle1/pose", self._sub_turtle1_pose_cb, 10)
        self.create_subscription(Pose, "/turtle2/pose", self._sub_turtle2_pose_cb, 10)

        self._turtle1_pose = Pose()
        self._turtle2_pose = Pose()
        self._tx = 0.0
        self._ty = 0.0

    def _sub_turtle1_pose_cb(self, msg:Pose):
        self._turtle1_pose = msg

        #self._tx = self._turtle1_pose.x
        #self._ty = self._turtle1_pose.y

        #self.get_logger().info(f"Translation-Vector Tx = {self._tx:.2f}, Ty={self._ty:.2f}")


    def _sub_turtle2_pose_cb(self, msg:Pose):
        self._turtle2_pose = msg

        self._tx = self._turtle2_pose.x - self._turtle1_pose.x
        self._ty = self._turtle2_pose.y - self._turtle1_pose.y

        self._td = sqrt(pow(self._tx,2) + pow(self._ty,2))

        self._theta = self._turtle2_pose.theta - self._turtle1_pose.theta
        rotation_matrix = [[cos(self._theta), - sin(self._theta)],
                           [sin(self._theta), + cos(self._theta)]]

        self.get_logger().info(f"Translation-Vector Tx = {self._tx:.2f}, Ty={self._ty:.2f}, Td={self._td:.2f}")
        self.get_logger().info(f"Rotations-Matrix: {rotation_matrix}")
        
def main():
    try: 
        rclpy.init()
        node = SimpleKinematics("SimpleKinematics")
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        print("Sie haben Strg+C gedrückt")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
     main()