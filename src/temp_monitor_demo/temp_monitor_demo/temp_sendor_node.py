from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32
import random
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class TempSensorNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._pub_temperatur = self.create_publisher(Float32, 'temperature', 10)
        
        self.declare_parameters(
            "",
            parameters=[
                ("hz", 2.0),    # Abtastrate in Hz
                ("mean", 25.0), # Mittelwert
                ("amp", 5.0),   # Amplitude
            ]
        )

        self._hz = self.get_parameter("hz").get_parameter_value().double_value
        self._mean = self.get_parameter("mean").get_parameter_value().double_value
        self._amp = self.get_parameter("amp").get_parameter_value().double_value#
        

        
        # Sobalt Parameter geaendert werden, wird die Callback Funktion aufgerufen
        self.add_on_set_parameters_callback(self.on_parameter_set_cb)
        
        self._timer_temperatur = self.create_timer(1.0/self._hz, self.timer_temperatur_cb)

    def timer_temperatur_cb(self):
        val = Float32()
        
        # wird jetzt in der Callback Funktion gemacht
        #self._hz = self.get_parameter("hz").get_parameter_value().double_value
        #self._mean = self.get_parameter("mean").get_parameter_value().double_value
        #self._amp = self.get_parameter("amp").get_parameter_value().double_value
        
        val.data = random.uniform(self._mean-self._amp,self._mean+self._amp)
        self._pub_temperatur.publish(val)
        
    def on_parameter_set_cb(self , param:list[Parameter]):
        for p in param: 
            if p.name == 'hz': 
                self._hz = float(p.value)
                self._timer_temperatur.cancel()
                self._timer_temperatur = self.create_timer(1.0/self._hz, self.timer_temperatur_cb)
            elif p.name == 'mean': self._mean = float(p.value)
            elif p.name == 'amp': self._amp = float(p.value)
        
        return SetParametersResult(successful=True) 

    def destroy_node(self):
        return super().destroy_node()
    
def main():
    node = None
    try:
        rclpy.init()
        try:
            node = TempSensorNode("temp_sensor")
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