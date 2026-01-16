#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler
import time

from motorctrl.diffdrive_config_node import DiffDriveConfigNode, set_hw_state

class SimpleVelocityController(DiffDriveConfigNode):

    def __init__(self, node_name="simple_velocity_controller"):
        super().__init__(node_name)
        
        self._wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self._wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self._wheel_radius)
        self.get_logger().info("Using wheel separation %d" % self._wheel_separation)

        self._left_wheel_prev_pos = 0.0
        self._right_wheel_prev_pos = 0.0
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        self._pub_wheel_cmd = self.create_publisher(Float64MultiArray, "velocity_controllers/commands", 10)
        self._sub_vel_cmd = self.create_subscription(Twist, "awdbot_controller/cmd_vel_unstamped", self.sub_cmd_vel_cb, 10)
        self._sub_joint_states = self.create_subscription(JointState,"joint_states", self.sub_joint_state_cb, 10)        
        
        # Pose des Roboters im World-Frame ("odom" Frame)
        # D.h. x, y und Theta zwischen den Frames odom und base_footprint
        self._pub_odom = self.create_publisher(Odometry, "awdbot_controller/odom", 10)

        self._matrix_speed_conversion = np.array([[self._wheel_radius/2, self._wheel_radius/2],
                                           [self._wheel_radius/self._wheel_separation, -self._wheel_radius/self._wheel_separation]])
        self.get_logger().info("The conversion matrix is %s" % self._matrix_speed_conversion)

        # Fill the Odometry message with invariant parameters
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = "odom"
        self._odom_msg.child_frame_id = "base_footprint"
        # zu Beginn ist der Roboter im Ursprung von odom Frame
        self._odom_msg.pose.pose.position.x = self._x
        self._odom_msg.pose.pose.position.y = self._y
        self._odom_msg.pose.pose.position.z = 0.0
        # zu Beginn ist der Roboter nicht rotiert (keine Orientierung)
        self._odom_msg.pose.pose.orientation.x = 0.0
        self._odom_msg.pose.pose.orientation.y = 0.0
        self._odom_msg.pose.pose.orientation.z = 0.0
        self._odom_msg.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self._transform_br = TransformBroadcaster(self)
        self._transform_stamped = TransformStamped()
        self._transform_stamped.header.frame_id = "odom"
        self._transform_stamped.child_frame_id = "base_footprint"

        self._prev_time = self.get_clock().now()

        self._timeout = self.declare_parameter("timeout", 0.5).get_parameter_value().double_value
        self._last_cmd = time.monotonic()
        self._stopped = True
        self._watchdog = self.create_timer(0.1, self._watchdog_cb)

    def _watchdog_cb(self):
        if (time.monotonic() - self._last_cmd) > self._timeout and not self._stopped:
            self._pub_wheel_cmd.publish(Float64MultiArray(data=[0,0,0,0]))
            self._stopped = True

    def sub_cmd_vel_cb(self, msg:Twist):
        # Differentielle kinematische Modell
        # Geschwindigkeit des Roboters (linedar v und Wineklgeschw. w) werden
        # in in Winkelgeschwindikeit der einzelnen Reifen umgerechnet!
        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self._matrix_speed_conversion), robot_speed) 

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [
            #Vorderreifen
            wheel_speed[1, 0], wheel_speed[0, 0],
            #Hinterreifen
            wheel_speed[1, 0], wheel_speed[0, 0]
            ]
        
        self._last_cmd = time.monotonic()
        self._stopped = False

        self.get_logger().info("Publishing wheel speed: %s" % wheel_speed_msg.data)
        self._pub_wheel_cmd.publish(wheel_speed_msg)
  
    def sub_joint_state_cb(self, msg:JointState):
        #Implemetniert das inverse differentielle kinematische Modell
        #Gegeben die Position der Räder, berechnet ihre Geschwindigkeiten
        #dann berechnet die Geschwindigkeit des Roboters im Roboter-Frame
        #und dann konvertiert es in den Worlf-Frame und veröffentlicht das TF

        #Änderung des Drehwinkels der Räder
        #Wir nehmen an, dass die Räder kein Rauschen haben
        dp_left = msg.position[1] - self._left_wheel_prev_pos
        dp_right = msg.position[0] - self._right_wheel_prev_pos
        dt = Time.from_msg(msg.header.stamp) - self._prev_time

        # Aktualisieren der vorherigen Position für die nächste Iteration
        self._left_wheel_prev_pos = msg.position[1]
        self._right_wheel_prev_pos = msg.position[0]
        self._prev_time = Time.from_msg(msg.header.stamp)

        # Berechnung der Rotationsgeschwindigkeit jedes Rades
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # Geschwindigkeit v des Roboters in X-Richtung und Winkelgeschwindigkeit w um die Z-Achse (Robot-Frame)
        linear = (self._wheel_radius * fi_right + self._wheel_radius * fi_left) / 2
        angular = (self._wheel_radius * fi_right - self._wheel_radius * fi_left) / self._wheel_separation

        # Positionsänderung in X-Richung des Robotes (Robot-Frame)
        d_s = (self._wheel_radius * dp_right + self._wheel_radius * dp_left) / 2
        # Winkeländerung um die Z-Achse des Robotes (Robot-Frame)
        d_theta = (self._wheel_radius * dp_right - self._wheel_radius * dp_left) / self._wheel_separation
        
        # Rotation (Robot-Frame -> World-Frame)
        self._theta += d_theta
        self._x += d_s * math.cos(self._theta)
        self._y += d_s * math.sin(self._theta)
        
        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self._theta)
        self._odom_msg.header.stamp = self.get_clock().now().to_msg()
        self._odom_msg.pose.pose.position.x = self._x
        self._odom_msg.pose.pose.position.y = self._y
        self._odom_msg.pose.pose.orientation.x = q[0]
        self._odom_msg.pose.pose.orientation.y = q[1]
        self._odom_msg.pose.pose.orientation.z = q[2]
        self._odom_msg.pose.pose.orientation.w = q[3]
        self._odom_msg.twist.twist.linear.x = linear
        self._odom_msg.twist.twist.angular.z = angular
        self._pub_odom.publish(self._odom_msg)

        # TF
        self._transform_stamped.transform.translation.x = self._x
        self._transform_stamped.transform.translation.y = self._y
        self._transform_stamped.transform.rotation.x = q[0]
        self._transform_stamped.transform.rotation.y = q[1]
        self._transform_stamped.transform.rotation.z = q[2]
        self._transform_stamped.transform.rotation.w = q[3]
        self._transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self._transform_br.sendTransform(self._transform_stamped)

    def _srv_search_motors_cb(self, request, response):
        set_hw_state(self, "RobotSystem", "unconfigured")
        response = super()._srv_search_motors_cb(request, response)
        set_hw_state(self, "RobotSystem", "configured")
        return response
    
    def _srv_set_motor_config_cb(self, request, response):
        set_hw_state(self, "RobotSystem", "unconfigured")
        response = super()._srv_set_motor_config_cb(request, response)
        set_hw_state(self, "RobotSystem", "configured")
        return response
    
    def save_ros2_control_parameters(self):
        """
        Speichert die ROS2-Control-Parameter in einer YAML-Datei.
        """
        params_dict = {}
    
    def destroy_node(self):
        return super().destroy_node()


def main():
    node = None
    try:
        rclpy.init()
        try:
            node = SimpleVelocityController("diffdrive_velocity_controller")
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
