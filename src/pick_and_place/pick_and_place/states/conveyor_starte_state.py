from __future__ import annotations
from ..state_machine import State # -> geht einen ortner zurück und dann in state_machine

from pick_and_place.pick_and_place_node import PickAndPlaceNode
from std_srvs import SetBool


class ConveyorStartState(State):
    """Prüft ob der Roboter OK ist und startet dannach den Normalbetrieb"""

    def __init__(self) -> None:
        super().__init__("START_CONVEYOR")
        self._counter: int = 0

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")
        
        request = SetBool.Request()
        request.data = True
        self._future = node._cli_conveyor.call_async(request)
        
    def tick(self, node) -> str | None:
        if self._future.done():
            response: SetBool = self._future.result()
            if response.success:
                return "WAIT_FOR_OBJECT"
        return None
            
    def on_exit(self, node) -> None:
        node.get_logger().info(f"EXIT {self.name}")