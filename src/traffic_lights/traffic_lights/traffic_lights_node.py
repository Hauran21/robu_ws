from rclpy.node import Node
import rclpy
from std_msgs.msg import Int8
from traffic_lights_interfaces.srv import SetTrafficLightsMode
from enum import Enum, auto

class TrafficLightsState(Enum):
    OFF = auto()
    RED = auto()
    YELLOW = auto()
    RED_YELLOW = auto()
    GREEN = auto()
    GREEN_BLINK = auto()
    YELLOW_BLINK = auto()
    
class TrafficLightsMode(Enum):
    STANDARD = 1 #Ohne Gelb-Phase
    STANDARD_RED_YELLOW = 2 #Mit Rot-Gelb-Phase
    OFF = 3 #Gelb blinkend
    CONSTRUCTION = 4 #ToDo

class TrafficLightsNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        self.declare_parameter("color_red", (255, 0, 0))
        self.declare_parameter("color_green", (0, 255, 0))
        self.declare_parameter("color_yellow", (255, 255, 0))
        
        self.declare_parameter("duration_red", 10.0)
        self.declare_parameter("duration_green", 3.0)
        self.declare_parameter("duration_yellow", 10.0)
        
        self.declare_parameter("duration_blink", 0.5)
        self.declare_parameter("repetitions_blink", 3)
        
        self.declare_parameter("mode", TrafficLightsMode.STANDARD.value)
        
        self._color_red = self.get_parameter("color_red").get_parameter_value().integer_array_value
        self._color_green = self.get_parameter("color_green").get_parameter_value().integer_array_value
        self._color_yellow = self.get_parameter("color_yellow").get_parameter_value().integer_array_value
        
        self._duration_red = self.get_parameter("duration_red").get_parameter_value().double_value
        self._duration_green = self.get_parameter("duration_green").get_parameter_value().double_value
        self._duration_yellow = self.get_parameter("duration_yellow").get_parameter_value().double_value
        
        self._duration_blink = self.get_parameter("duration_blink").get_parameter_value().double_value
        self._repetitions_blink = self.get_parameter("repetitions_blink").get_parameter_value().integer_value   
        self._mode = TrafficLightsMode(self.get_parameter("mode").get_parameter_value().integer_value)
        
        self._tl_state = TrafficLightsState.OFF
        self._blink_counter = 0
        
        self._pub_tl_state = self.create_publisher(Int8, '/tl_state', 10)
        # das "/" beim topic macht dass das topic auch wenn man es öfter startet immer so heißt. Wenn man es weglässt, kommt de NodeName davor.
        
        self.create_service(
            srv_type=SetTrafficLightsMode,
            srv_name='/set_traffic_light_mode',
            callback=self._srv_set_tl_mode_cb
        )
        
        self._timer_tl = self.create_timer(1.0, self._timer_tl_cb)
        
    def _srv_set_tl_mode_cb(self, request:SetTrafficLightsMode.Request, response:SetTrafficLightsMode.Response) -> SetTrafficLightsMode.Response:
        
        self._mode = TrafficLightsMode(request.mode)
        
        response.mode = self._mode.value
        response.success = True
        
        return response
        
    def _timer_tl_cb(self): 
        self._set_traffic_light_state()
        self.get_logger().info(f"Ampelzustand: {self._tl_state.name}")
        self._pub_tl_state.publish(Int8(data=self._tl_state.value))
    
    def _set_traffic_light_state(self):
        #Modus der Verkehrsampel (1...Standart, 2...Standart mit Gelb/Rot, 3...Außer Betrieb, 4...Baustellenmodus)
        if self._mode == TrafficLightsMode.STANDARD or self._mode == TrafficLightsMode.STANDARD_RED_YELLOW:
            if self._tl_state == TrafficLightsState.OFF:
                self._timer_tl.destroy()
                self._timer_tl = self.create_timer(self._duration_red, self._timer_tl_cb)
                self._tl_state = TrafficLightsState.RED
            elif self._tl_state == TrafficLightsState.RED:
                if self._mode == TrafficLightsMode.STANDARD_RED_YELLOW:
                    self._timer_tl.destroy()
                    self._timer_tl = self.create_timer(self._duration_yellow, self._timer_tl_cb)
                    self._tl_state = TrafficLightsState.RED_YELLOW
                else:
                    self._timer_tl.destroy()
                    self._timer_tl = self.create_timer(self._duration_green, self._timer_tl_cb)
                    self._tl_state = TrafficLightsState.GREEN
            elif self._tl_state == TrafficLightsState.RED_YELLOW:
                self._timer_tl.destroy()
                self._timer_tl = self.create_timer(self._duration_green, self._timer_tl_cb)
                self._tl_state = TrafficLightsState.GREEN
            elif self._tl_state == TrafficLightsState.GREEN:
                self._timer_tl.destroy()
                self._timer_tl = self.create_timer(self._duration_yellow, self._timer_tl_cb)
                self._tl_state = TrafficLightsState.GREEN_BLINK
                self._blink_counter = 0
            elif self._tl_state == TrafficLightsState.GREEN_BLINK:
                if self._blink_counter < 2 * self._repetitions_blink -1:
                    self._blink_counter += 1
                else:
                    self._timer_tl.destroy()
                    self._timer_tl = self.create_timer(self._duration_red, self._timer_tl_cb)
                    self._tl_state = TrafficLightsState.YELLOW                
            elif self._tl_state == TrafficLightsState.YELLOW:
                self._timer_tl.destroy()
                self._timer_tl = self.create_timer(self._duration_red, self._timer_tl_cb)
                self._tl_state = TrafficLightsState.RED
            else:
                self.get_logger().error("Ungültiger Zustand der Verkehrsampel im Standardmodus.Darf nicht eintreten")
                self._tl_state = TrafficLightsState.YELLOW_BLINK
        elif self._mode == TrafficLightsMode.OFF:
            self._tl_state = TrafficLightsState.OFF
        elif self._mode == TrafficLightsMode.CONSTRUCTION:
            pass

            
    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = TrafficLightsNode("traffic_lights")
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