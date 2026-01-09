from __future__ import annotations
from typing import TYPE_CHECKING

from pick_and_place.state_machine import State

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class AnalyzeObject(State):
    """
    Analysiert das Objekt (z.B. per Kamera).
    Hier nur Dummy: setzt Farbe und Pose.
    """
    name:str="ANALYZE_OBJECT"
    def __init__(self) -> None:
        super().__init__(AnalyzeObject.name)

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {AnalyzeObject.name}: starte Kamera-Auswertung (Dummy).")

        # TODO: hier später Kamera/Computer-Vision integrieren.
        # Kamera-Service aufrufen -> Die Kamera erstellt ein Bild vom Objekt

    def tick(self, node:PickAndPlaceNode) -> str | None:

        #1. ) Future-Objekt der Kamera-Service abfragen und warten
        #     bis die Kamera ein Ergebnis liefert
        #2. ) Analyse des Bilds (mit MachineLearning oder mit OpenCV)
        #3. ) Bestimmung der Pose und Farbe des Objekts

        #Statt 1 bis 3 -> Fake:
        node.get_logger().info(
            f"Analyse fertig: Farbe={node.object_color}, Pose={node.object_pose}"
        )
        return "MOVE_TO_PICK"

    def on_exit(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"EXIT {AnalyzeObject.name}")
        # Speicherung der Pose und Farbe als Eigenschaft meines Kontexts (Node)
        # Fake-Ergebnis speichern
        # Für die Demo setzen wir Dummy-Werte:
        node.object_color = "red"
        node.object_pose = "dummy_pose"
