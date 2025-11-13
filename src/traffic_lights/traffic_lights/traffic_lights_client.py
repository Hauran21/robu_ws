from rclpy.node import Node
import rclpy
from traffic_lights_interfaces.srv import SetTrafficLightsMode
from traffic_lights.traffic_lights_node import TrafficLightsMode

class TrafficLightClient(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        # match the server's service name (server registers '/set_traffic_light_mode')
        self._cli_set_traffic_lights = self.create_client(SetTrafficLightsMode, '/set_traffic_light_mode')

        while not self._cli_set_traffic_lights.wait_for_service(1.0):
            self.get_logger().warning("Gernot wartet auf den Service :(")

        self.get_logger().info("Gernot hat den Service gefunden :)")
        
    def send_request(self, mode:TrafficLightsMode=TrafficLightsMode.STANDARD):
        request = SetTrafficLightsMode.Request()
        request.mode = mode.value

        future = self._cli_set_traffic_lights.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response:SetTrafficLightsMode.Response = future.result()    
        self.get_logger().info(f"mode: {request.mode} -> {response.mode}, {response.success}")

    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = TrafficLightClient("traffic_light_client")
            node.send_request(TrafficLightsMode.STANDARD_RED_YELLOW)
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return

        #rclpy.spin(node)
        rclpy.spin_once(node) #ROS Loop wird nur einmal durchgeführt

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