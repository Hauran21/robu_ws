from __future__ import annotations
from ..state_machine import State # -> geht einen ortner zurück und dann in state_machine

from pick_and_place.pick_and_place_node import PickAndPlaceNode
from std_srvs import SetBool


class MoveToPlace(State):
    """Prüft ob der Roboter OK ist und startet dannach den Normalbetrieb"""

    def __init__(self) -> None:
        super().__init__("MOVE_TO_PLACE")
        self._counter: int = 0

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")
        
        
        
    def tick(self, node:PickAndPlaceNode) -> str | None:
        if self._future.done():
            response: SetBool = self._future.result()
            if response.success:
                return "RELEASE_OBJECT"
        return None
            
    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {self.name}")