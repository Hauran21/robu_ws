from rclpy.node import Node
import rclpy
import sys
import os
from datetime import datetime

from cv_bridge import CvBridge
import cv2

from example_camera_inteface.srv import TakePhoto


class CameraServiceClientNode(Node):
    def __init__(self, node_name: str = 'camera_service_client'):
        super().__init__(node_name)
        self._client = self.create_client(TakePhoto, 'example_camera_service')
        self._bridge = CvBridge()

        # wait for service
        while not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Warte auf service example_camera_service...')
        self.get_logger().info('Service example_camera_service ist verfügbar.')

    def send_request(self) -> rclpy.executors.Future:
        req = TakePhoto.Request()
        req.take_photo = True
        return self._client.call_async(req)

    def handle_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        if resp is None:
            self.get_logger().error('Service returned no response')
            return

        img_msg = resp.camera_image
        if img_msg is None:
            self.get_logger().error('Response contains no camera_image')
            return

        try:
            cv_image = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image message: {e}')
            return

        out_dir = '/tmp'
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(out_dir, f'take_photo_{ts}.png')

        try:
            cv2.imwrite(out_path, cv_image)
            self.get_logger().info(f'Image saved to {out_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {e}')


def main(argv=sys.argv[1:]):
    node = None
    try:
        rclpy.init()
        node = CameraServiceClientNode()

        future = node.send_request()
        future.add_done_callback(node.handle_response)

        # spin until the future completes (or CTRL+C)
        rclpy.spin_until_future_complete(node, future)

    except KeyboardInterrupt:
        print('Sie haben STRG+C gedrückt!')
    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info(f'Node {node.get_name()} wird beendet!')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()