from __future__ import annotations
from ..state_machine import State # -> geht einen ortner zurück und dann in state_machine

from pick_and_place.pick_and_place_node import PickAndPlaceNode
from std_srvs import SetBool


class ConveyorWaitForObject(State):
    """Prüft ob der Roboter OK ist und startet dannach den Normalbetrieb"""

    def __init__(self) -> None:
        super().__init__("WAIT_FOR_OBJECT")
        self._counter: int = 0

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")
        
        
        
    def tick(self, node) -> str | None:
        if node.light_barrier_detected == True:
            return "STOP_CONVEYOR"
        return None
            
    def on_exit(self, node) -> None:
        node.get_logger().info(f"EXIT {self.name}")