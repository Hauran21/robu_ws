from __future__ import annotations
from typing import TYPE_CHECKING

from pick_and_place.state_machine import State
from pick_and_place.states.conveyor_stop_state import StopConveyor

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class WaitForObject(State):
    """
    Warten, bis die Lichtschranke ein Objekt meldet.
    """
    name="WAIT_FOR_OBJECT"
    def __init__(self) -> None:
        super().__init__(WaitForObject.name)

    def on_enter(self, node: PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}: warte auf Objekt an der Lichtschranke.")

    def tick(self, node: PickAndPlaceNode) -> str | None:
        if node.object_detected:
            node.get_logger().info(f"Objekt erkannt → {StopConveyor.name}")
            return StopConveyor.name

        return None
