from __future__ import annotations
from ..state_machine import State # -> geht einen ortner zurück und dann in state_machine

from pick_and_place.pick_and_place_node import PickAndPlaceNode

class IdleState(State):
    """Prüft ob der Roboter OK ist und startet dannach den Normalbetrieb"""

    def __init__(self) -> None:
        super().__init__("IDLE_STATE")
        self._counter: int = 0

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")
        
    def tick(self, node:PickAndPlaceNode) -> str | None:
        # Hier könnten sicherheitsrelevante Bedingungen geprüft werden
        # z.B. Zugang zum Roboter sind versperrt. Alle Sensoren liefern Daten
        return "START_CONVEYOR"

    def on_exit(self, node) -> None:
        node.get_logger().info(f"EXIT {self.name}")