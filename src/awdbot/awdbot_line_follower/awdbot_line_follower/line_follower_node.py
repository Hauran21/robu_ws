from rclpy.node import Node
import rclpy

import cv2
import os

def test_track_detection():
    frame = cv2.imread('/home/robot/work/robu_ws/src/awdbot/awdbot_line_follower/test/testbild.png')
    # cv2.imshow("testbild", frame)
    # cv2.waitKey(0)

    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("testbild", gray)
    # cv2.waitKey(0)
    
    mask = cv2.inRange(gray, 0, 100)
    # cv2.imshow("testbild", mask)
    # cv2.waitKey(0)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        for contour in contours:
            if cv2.contourArea(contour) > 100: #100pixel große fläche
                frame_contour = cv2.drawContours(frame, [contour], 0, (255, 0, 0), 2, )
                cv2.imshow("Testbild", frame_contour)#
                cv2.waitKey(0)
            
            
    cv2.destroyAllWindows()

class LineFollower(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        # TODO: Alle notwendige Einstellungen als Parameter hinterlegen 
        # z.B. Regeleinstellungen {PID-Werte}, 
        # max. Geschwindigkeit, 
        # welche Streckenfarbe soll verfolgt werden, 
        # mindestabstand zu objekten
        
        # TODO: 
        # Subscriber für das Kamerabid -> /camera/image_raw
        # Publisher für die Geschwindigkeitsbefehle -> /cmd_vel
        # Subscriber für Abstandssensoren -> /scan
        

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