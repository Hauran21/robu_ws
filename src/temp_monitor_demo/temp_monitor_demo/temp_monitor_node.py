from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32, String

class TempMonitorNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        self.declare_parameters(
            "",
            parameters=[
                ("threshold", 27.0),
            ]
        )
        
        self._threshold = self.get_parameter("threshold").get_parameter_value().double_value
        
        self.create_subscription(Float32, 'temperature', self._sub_temperatur_cb, 10)
        
        self._pub_temperatur_alert = self.create_publisher(String, 'temperatur_alert', 10)
    
    def _sub_temperatur_cb(self, msg:Float32):
        val_temp = msg.data
        
        self.get_logger().info(f"Aktuelle Temperatur: {val_temp:.2f}")
        
        if val_temp > self._threshold:
            self.get_logger().warn(f"Temperaturalarm! Wert: {val_temp}")
            self._pub_temperatur_alert.publish(String(data=f"Temperaturalarm! Wert: {val_temp}"))

    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = TempMonitorNode("temp_monitor")
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Sie haben STRG+C gedrückt!")

    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info(f"Node {node.get_name()} wird beendet!")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()