import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import random

class DistanceSensorNode(Node):
    
    def __init__(self, node_name="distance_sensor"):
        super().__init__(node_name)
        
        self._pub_distance = self.create_publisher(Float32, "distance", 10) # "Float32" -> ist der Ros Name für float 
        self.create_timer(.5, self._timer_distance_cb)
        
    def _timer_distance_cb(self):
        val = Float32()
        
        val.data = random.uniform(0.0, 1.0)

        self._pub_distance.publish(val)  
        self.get_logger().info(f"Distance: {val.data:3}") #gibt val.data aus mit 3 nachkommastellen        
  
        
class ObstaceAvoidanceNode(Node):
    def __init__(self, node_name='obstacle_avoidance'):
        super().__init__(node_name)
        
        self._sub_distance = self.create_subscription(Float32, "distance", self._sub_distance_cb, 10)
        self._sub_distance_min = 0.3
        
    
    def _sub_distance_cb(self, msg:Float32):
        if msg.data < self._sub_distance_min:
            self.get_logger().warn("Kollision steht unmittelbar bevor!")
        self.get_logger().info(f"Distance: {msg.data:3}")
          


def main_distance_sensor():
    try:
        rclpy.init()
        node = DistanceSensorNode()
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        print("Sie haben STRG+C gedrückt!")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def main_obstacle_avoidance():
    try:
        rclpy.init()
        node = ObstaceAvoidanceNode()
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        print("Sie haben STRG+C gedrückt!")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main_distance_sensor()