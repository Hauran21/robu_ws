from __future__ import annotations
from ..state_machine import State # -> geht einen ortner zurück und dann in state_machine

from pick_and_place.pick_and_place_node import PickAndPlaceNode
from std_srvs import SetBool


class StopConveyor(State):
    """Das Förderband wird gestoppt."""

    def __init__(self) -> None:
        super().__init__("STOP_CONVEYOR")

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")
        
        request = SetBool.Request()
        request.data = False
        self._future = node._cli_conveyor.call_async(request)

        
    def tick(self, node:PickAndPlaceNode) -> str | None:
        if self._future.done():
            response: SetBool = self._future.result()
            if response.success:
                return "STOP_CONVEYOR"
        return None
            
    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {self.name}")