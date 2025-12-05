from rclpy.node import Node
import rclpy

from pick_and_place.state_machine import StateMachine
from std_srvs.srv import SetBool
from std_msgs.msg import Bool

class PickAndPlaceNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        # Merker erzeugen
        self.vacum_ok :bool = False
        self.light_barrier_detected :bool  = False
        self.conveyor_running :bool = False
        
        # State-Machine erzeugen
        # Als Kontext wird der Node übergeben
        # IN unserer Klasse PickAndPlaceNOde befinden sich alle INfos (Merker, Publisher, Services etc.) damit die State Machine arbeitet
        self.sm = StateMachine(self)
        
        # States zur State-Machine hinzufügen
        # self.sm.add_state(...)   
        self.sm.set_initial_state("IDLE")
        
        self._cli_conveyor = self.create_client(SetBool, 'conveyor/set_running')
        self.get_logger().info("Warte auf Service conveyor/set_running...")
        self._cli_conveyor.wait_for_service()
              
        self.create_subscription(
            Bool,
            'light_barrier/detected',
            self._sub_lb_detected_cb,
            10
        )    
        
        self.create_subscription(
            Bool, 
            "vacuum/detected",
            self._sub_vacuum_detected_cb,
            10
        )
        
        # Dieser Timer wird beutzt um die States abzuarbeiten
        self._timer_sm = self.create_timer(0.1, self._timer_sm_cb)

    def _timer_sm_cb(self):
        self.sm.step()
    
    def _sub_lb_detected_cb(self, msg: Bool):
        # In einem zweiten Programm läuft ein publisher. 
        # Dieser prüft die Werte der Lichtschranke
        # Wenn sich der Zustand der Lichtschranke ändert sendet der Publisher True 
        # (Objekt blockiert die Lichtschranke)
        
        # Wir merken uns nun hier den Zustand der Lichtschranke für den WaitForObjectState
        self.light_barrier_detected = msg.data

    def _sub_vacuum_detected_cb(self, msg:Bool):
        self.vacuum_ok = msg.data

    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = PickAndPlaceNode("pick_and_place")
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