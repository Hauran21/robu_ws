from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image

from example_camera_inteface.srv import TakePhoto


class CameraServiceClient(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        self._image_publisher = self.create_publisher(
            Image, '/image_raw', 10
        ) 
        
        self._client_camera_service = self.create_client(
            TakePhoto, "example_camera_service"
        )
        
        while not self._client_camera_service.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning("warten auf service example_camera_service...")
        self.get_logger().info("Service example_camera_service ist verfügbar.")
        
        self._req_camera_service = TakePhoto.Request()
    
    def send_request(self, take_image: bool):
        # field name in TakePhoto.srv is `take_photo`
        self._req_camera_service.take_photo = take_image
        return self._client_camera_service.call_async(self._req_camera_service)
       
    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = CameraServiceClient("camera_service_client")
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return

        future = node.send_request(True)
        #future.add_done_callback(node._handle_response)
        
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        node.get_logger().info(f'Camera service response received.')

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