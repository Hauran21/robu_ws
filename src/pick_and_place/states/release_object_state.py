from __future__ import annotations
from typing import TYPE_CHECKING

from pick_and_place.state_machine import State

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode


class ReleaseObject(State):
    """
    Öffnet den Greifer / schaltet Vakuum aus.
    """
    name="RELEASE_OBJECT"
    def __init__(self) -> None:
        super().__init__(ReleaseObject.name)

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(
            f"ENTER {ReleaseObject.name}: Objekt ablegen, Greifer öffnen / Vakuum aus."
        )
        # TODO: Greifer öffnen / Vakuum deaktivieren

    def tick(self, node:PickAndPlaceNode) -> str | None:
        if node.vacuum_ok:
            return None
        
        return "MOVE_TO_HOME"

    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {ReleaseObject.name}")

