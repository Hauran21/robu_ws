from rclpy.node import Node
import rclpy
import cv2
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge # Brücke zwischen OpenCV und ROS2 Nachrichten
from sensor_msgs.msg import Image

from example_camera_inteface.srv import TakePhoto

class CameraServiceNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        self._cap_publisher = self.create_publisher(
            Image, '/image_raw', 10
        ) 
        
        self.declare_parameter("resolution",
                               [640, 480],
                               ParameterDescriptor(
                                description="Resolution of the camera image as [width, height]"
                               ))
        
        self.declare_parameter("device",
                               0, 
                               ParameterDescriptor(
                                description="Camera device index [e.g., 0 for /dev/video0]"
                               ))
        
        self._param_device = self.get_parameter("device").get_parameter_value().integer_value
        self._param_resolution = self.get_parameter("resolution").get_parameter_value().integer_array_value
        
        self._srv_take_photo = self.create_service(
            TakePhoto, "take_photo", self.take_photo_cb
        )
        
        self._cvbridge = CvBridge()
        self._cap = cv2.VideoCapture("/dev/video" + str(self._param_device))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._param_resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._param_resolution[1])
        
        if not self._cap.isOpened():
            self.get_logger().error(f"Konnte Kamera /dev/video{self._param_device} nicht öffnen.")
            raise Exception("Kamera konnte nicht geöffnet werden.")
        
    def take_photo_cb(self, request:TakePhoto.Request, response:TakePhoto.Response) -> TakePhoto.Response:
        
        ret, frame = self._cap.read()
        
        msg = self._cvbridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        self._cap_publisher.publish(msg)
        response.img = msg
        #response.msg.data = "Photo taken and published to /image_raw"

        return response

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
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