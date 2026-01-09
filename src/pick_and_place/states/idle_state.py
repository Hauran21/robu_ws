from __future__ import annotations
from typing import TYPE_CHECKING

from pick_and_place.state_machine import State
from rclpy.node import Node

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class Idle(State):
    """Prüft ober der Roboter OK ist und startet danach den Normalbetrieb"""

    name:str="IDLE_STATE"
    def __init__(self) -> None:
        super().__init__(Idle.name)

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")

    def tick(self, node:PickAndPlaceNode) -> str | None:
        # Hier könnten sicherheitsrelevante Bedingungen geprüft werden
        # z.B. Zugänge zum Roboter sind versperrt, allen Sensoren liefern Daten (Lichtschranke, 
        # Vacuum Sensor, Not-Halt ist ok)
        return "CONVEYOR_START"

    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {self.name}")