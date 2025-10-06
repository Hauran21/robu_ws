from rclpy.node import Node
import rclpy, sys

from example_add_interface.srv import AddTwoInts

class ExampleAddClientNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._client_add_two_ints = self.create_client(AddTwoInts, "add_two_ints")

        while not self._client_add_two_ints.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("warten auf service add_two_ints...")
        self.get_logger().info("Service add_two_ints ist verfügbar.")
        
        self._req_add_two_ints = AddTwoInts.Request()
        
    def send_request(self, a: int, b: int):
        self._req_add_two_ints = a
        self._req_add_two_ints = b
        return self._client_add_two_ints.call_async(self._req_add_two_ints)
    
    def destroy_node(self):
        return super().destroy_node()
    

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = ExampleAddClientNode("example_add_client")
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return
        
        future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        node.get_logger().info(f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')


        #rclpy.spin(node)

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