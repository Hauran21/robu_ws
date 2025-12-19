from __future__ import annotations
from ..state_machine import State # -> geht einen ortner zurück und dann in state_machine

from pick_and_place.pick_and_place_node import PickAndPlaceNode
from std_srvs import SetBool


class WaitForObjectState(State):
    """Wartet bis die Lichtschranke ein Objekt detektiert."""

    def __init__(self) -> None:
        super().__init__("WAIT_FOR_OBJECT")

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")
        self._counter_on_error = 0
        
    def tick(self, node:PickAndPlaceNode) -> str | None:
        node.get_logger().info(f"Tick wait_for_object")
        self._counter_on_error += 1
        if self._counter_on_error > 600:
            pass
            #TODO Error state einfügen
            #return "ERROR"
        if node.light_barrier_detected == True:
            return "STOP_CONVEYOR"
        return None
            
    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {self.name}") 
