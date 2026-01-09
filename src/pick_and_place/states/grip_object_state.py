from __future__ import annotations
from typing import TYPE_CHECKING

from pick_and_place.state_machine import State

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class GripObject(State):
    """
    Schließt den Greifer / schaltet Vakuum ein
    und wartet, bis der Vakuumsensor 'grip_ok' meldet.
    """
    name:str="GRIP_OBJECT"
    def __init__(self) -> None:
        super().__init__(GripObject.name)

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {GripObject.name}: Greifer schließen / Vakuum einschalten.")
        # TODO: Topic oder Service zum Greifer/Vakuum-Controller schicken.

    def tick(self, node:PickAndPlaceNode) -> str | None:
        if node.vacuum_ok:
            node.get_logger().info("Vakuum OK → MOVE_TO_PLACE")
            return "MOVE_TO_PLACE"

        return None

    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {GripObject.name}")
