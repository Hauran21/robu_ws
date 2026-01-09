from __future__ import annotations
from typing import TYPE_CHECKING

from pick_and_place.state_machine import State

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class MoveToHome(State):
    """
    Fährt den Roboterarm in die Home-Position.
    """
    name="MOVE_TO_HOME"
    def __init__(self) -> None:
        super().__init__(MoveToHome.name)

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info("ENTER MOVE_TO_HOME: Roboter fährt in Home-Position (Dummy).")
        # TODO: MoveIt: Home-Position anfahren

    def tick(self, node:PickAndPlaceNode) -> str | None:
        return "START_CONVEYOR"

    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {MoveToHome.name}")
