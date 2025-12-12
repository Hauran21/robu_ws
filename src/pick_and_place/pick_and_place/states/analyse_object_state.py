from __future__ import annotations
from ..state_machine import State # -> geht einen ortner zurück und dann in state_machine

from pick_and_place.pick_and_place_node import PickAndPlaceNode
from std_srvs import SetBool


class AnalyzeObject(State):
    """Prüft ob der Roboter OK ist und startet dannach den Normalbetrieb"""

    def __init__(self) -> None:
        super().__init__("ANALYZE_OBJECT")
        self._counter: int = 0

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")
        
        self.object_color = "red"  # Platzhalter für Farberkennung
        self.object_pose = (0.5, 0.0, 0.2)  # Platzhalter für Objekterkennung
        
    def tick(self, node) -> str | None:
        if self.object_color != None and self.object_pose != None:
            return "MOVE_PACKAGE"
        return None
            
    def on_exit(self, node) -> None:
        node.get_logger().info(f"EXIT {self.name}")