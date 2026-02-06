from rclpy.node import Node
import rclpy

import cv2
import os

from math import radians
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool

from cv_bridge import CvBridge # Ros Nachricht vom Typen Image in ein OpenCv Image um

def test_track_detection():
    frame = cv2.imread('/home/robot/work/robu_ws/src/awdbot/awdbot_line_follower/test/testbild.png')
    # cv2.imshow("testbild", frame)
    # cv2.waitKey(0)

    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("testbild", gray)
    # cv2.waitKey(0)
    
    mask = cv2.inRange(gray, 0, 100) # Maske => hat nur 2 Werte da/nicht da
    # cv2.imshow("testbild", mask)
    # cv2.waitKey(0)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        for contour in contours:
            if cv2.contourArea(contour) > 100: #100pixel große fläche
                frame_contour = cv2.drawContours(frame, [contour], 0, (255, 0, 0), 2, )

                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                
                frame_contour = cv2.circle(frame_contour, (cx, cy), 10, (255, 0, 0), 2)            
            
                cv2.imshow("Testbild", frame_contour)#
                cv2.waitKey(0)

    cv2.destroyAllWindows()
    
def get_track_center(frame, gray_lower_value = 0, gray_upper_value = 100, min_area = 100) -> tuple[int , int] | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, gray_lower_value, gray_upper_value) 
    contours, j = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    cnt_idx = -1
    
    if contours:
            for i, contour in enumerate(contours):
                cont_area = cv2.contourArea(contour)
                if cont_area > min_area:
                    if cont_area > max_area: 
                        max_area = cont_area
                        cnt_idx = i

            if cnt_idx >= 0:
                contour = contours[cnt_idx]
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00']) #integer -> es gibt nur ganze Pixel
                    cy = int(M['m01'] / M['m00'])

                return (cx, cy)
    return None

class LineFollower(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        # TODO: Alle notwendige Einstellungen als Parameter hinterlegen 
        # z.B. Regeleinstellungen {PID-Werte}, 
        # max. Geschwindigkeit, 
        # welche Streckenfarbe soll verfolgt werden, -> PLF
        # mindestabstand zu objekten -> PLF
        
        # TODO: 
        # Subscriber für das Kamerabid -> /camera/image_raw
        # Publisher für die Geschwindigkeitsbefehle -> /cmd_vel
        # Subscriber für Abstandssensoren -> /scan
        
        self._srv_enable_line_follower = self.create_service(SetBool, "enable_linefollwer", self._srv_service_enable_line_follower_cb)
        
        self.declare_parameter("linear_speed_max", 0.2) # m/s
        self.declare_parameter("angular_speed_max", radians(60)) # grad/s
        
        self._sub_image = self.create_subscription(Image, "/camera/image_raw", self._sub_image_cb, 10)
        self._pub_cmd_vel = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self._pub_image_center = self.create_publisher(Image, "image_center", 10)
        
        self._cv_bridge = CvBridge() # Schnittstelle ROS <-> OpenCv 
        
        self._enable_line_follower = False

    def _srv_service_enable_line_follower_cb(self, req:SetBool.Request, resp:SetBool.Response) -> SetBool.Response:
        self._enable_line_follower = req.data
        if self._enable_line_follower:
            resp.message = "Line Follower wurde gestartet!"
        else: 
            resp.message = "Line Follower wurde beendet!"
            
        resp.success = True
        return resp

    def _sub_image_cb(self, msg:Image):
        
        frame = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        ret = get_track_center(frame)
        
        vel = TwistStamped() # Leer => alle Geschwindigkeiten sind auf 0

        if ret is None:
            self.get_logger().warn("Keine Strecke gefunden")
            self._pub_cmd_vel.publish(vel)
            return
        
        cx, cy = ret # Entpacken des Ergebnisses
        width = frame.shape[1]

        vel.twist.linear.x = self.get_parameter("linear_speed_max").get_parameter_value().double_value
        
        frame = self._create_marker_on_image(frame, (cx, cy), f"TC: {cx}, {cy}")
        msg_image = self._cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        self._pub_image_center.publish(msg_image)
        
        if not self._enable_line_follower:
            return

        if cx < width/2*0.9: # Strecke ist links vom Bild
            vel.twist.angular.z = self.get_parameter("angular_speed_max").get_parameter_value().double_value
        elif cx > width/2*1.1: # Strecke ist rechts vom Bild
            vel.twist.angular.z = - self.get_parameter("angular_speed_max").get_parameter_value().double_value
        else: # Strecke ist ca. in der Mitte
            pass
        
        self._pub_cmd_vel.publish(vel)

    def _create_marker_on_image(self, frame, point, text):
        cv2.circle(frame, point, 10, (255, 0, 0), 3)
        font = cv2.FONT_HERSHEY_COMPLEX
        font_scale = 1
        color = (255, 0, 0)
        thickness = 2
        
        # TODO: Test so ausgeben, dass dieser sich innerhalb des Bildes befindet!
        image = cv2.putText(frame, text, point, font, font_scale, color, thickness, cv2.LINE_AA)
        return image

    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = LineFollower("line_follower")
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
    test_track_detection()