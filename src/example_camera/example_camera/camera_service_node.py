from rclpy.node import Node
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from example_camera_inteface.srv import TakePhoto

class CameraServiceNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        self._image_publisher = self.create_publisher(
            Image, '/image_raw', 10
        ) 
        
        self._srv_take_photo = self.create_service(
            TakePhoto, "example_camera_service", self.take_photo_cb
        )
        
        self.declare_parameter("resolution", [640, 480])
        self.declare_parameter("device", 0)
        
        self._device = self.get_parameter("device").get_parameter_value().integer_value
        self._resolution = self.get_parameter("resolution").get_parameter_value().integer_array_value
        
      
        
    def take_photo_cb(self, request:TakePhoto.Request, response:TakePhoto.Response) -> TakePhoto.Response:
        if request.take_photo:
            self.image = cv2.VideoCapture("/dev/video" + str(self._device))
            self.bridge = CvBridge()

            self.image.set(cv2.CAP_PROP_FRAME_WIDTH, self._resolution[0])
            self.image.set(cv2.CAP_PROP_FRAME_HEIGHT, self._resolution[1])

            ret, frame = self.image.read()
            
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            self._image_publisher.publish(msg)
            response.camera_image = msg

            try:
                self.image.release()
            except Exception:
                pass

            self.get_logger().info("Foto aufgenommen")
        else:
            self.get_logger().warn("Kein Foto aufgenommen")

        return response

    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = CameraServiceNode("camera_service_node")
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