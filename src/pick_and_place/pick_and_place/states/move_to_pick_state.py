from __future__ import annotations
from typing import TYPE_CHECKING

from pick_and_place.state_machine import State

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class MoveToPick(State):
    """
    Fährt mit dem Roboterarm zur Greifposition.
    Hier nur als Platzhalter - MoveIt-Aufruf kann später ergänzt werden.
    """
    name="MOVE_TO_PICK"
    def __init__(self) -> None:
        super().__init__(MoveToPick.name)

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {MoveToPick.name}: Roboter fährt zur Pick-Position (Dummy).")
        # TODO: MoveIt: Plan + Execute zur Pick-Position basierend auf node.object_pose

    def tick(self, node:PickAndPlaceNode) -> str | None:
        node.get_logger().info(f"TICK {MoveToPick.name}")
        # Für die Demo: wir tun so, als wäre der Move sofort fertig
        # Prüfen ob die Pick-Pose erreicht wurde
        # -> Greifer aktivieren!
        return "GRIP_OBJECT"

    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {MoveToPick.name}")
