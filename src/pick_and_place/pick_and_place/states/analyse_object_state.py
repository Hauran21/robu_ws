from __future__ import annotations
from ..state_machine import State # -> geht einen ortner zurück und dann in state_machine

from pick_and_place.pick_and_place_node import PickAndPlaceNode
from std_srvs import SetBool


class AnalyzeObject(State):
    """Kamera analysiert das detektierte Objekt"""

    def __init__(self) -> None:
        super().__init__("ANALYZE_OBJECT")

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {self.name}")
        # Kamera SErvice aufruf >
        # Kamera erstellt ein Bild vom Objekt
        
    def tick(self, node:PickAndPlaceNode) -> str | None:
        # 1.)Future-Objekt der Kamera abfragen und warten
        #    bis die Kamera ein Ergebnis liefert
        # 2.) Analysie des Bildes (Mit MachineLearning oder mit OpenCV)
        # 3.) Farbe und Pose de Objekts bestimmen
        
        return "MOVE_TO_PICK"

    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {self.name}")
        #Speicherung des Pose unf Farbe als Eigenschaft meines Kontexts (Node)