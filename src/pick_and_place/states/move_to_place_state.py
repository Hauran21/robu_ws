from __future__ import annotations
from typing import TYPE_CHECKING

from pick_and_place.state_machine import State

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class MoveToPlace(State):
    """
    Fährt zur passenden Box abhängig von der Objektfarbe.
    """
    name="MOVE_TO_PLACE"
    def __init__(self) -> None:
        super().__init__(MoveToPlace.name)

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(
            f"ENTER {MoveToPlace.name}: fahre zur Box für Farbe {node.object_color} (Dummy)."
        )
        # TODO: MoveIt: Zielpose abhängig von node.object_color auswählen und anfahren.

    def tick(self, node:PickAndPlaceNode) -> str | None:
        # Demo: wir tun so, als wäre der Move sofort fertig
        #Prüfen ob die Pick-Pose erreicht wurde
        #-> Greifer aktivieren!
        return "RELEASE_OBJECT"

    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {MoveToPlace.name}")
