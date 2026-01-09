from __future__ import annotations
from typing import TYPE_CHECKING

import rclpy
from std_srvs.srv import SetBool

from pick_and_place.state_machine import State

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class StartConveyor(State):
    """
    Startet das Fließband über einen asynchronen Service-Call
    und bereitet den nächsten Zyklus vor.
    """
    name="START_CONVEYOR"
    def __init__(self) -> None:
        super().__init__(StartConveyor.name)
        self._future: rclpy.task.Future | None = None

    def on_enter(self, node:PickAndPlaceNode) -> None:
        node.get_logger().info(f"ENTER {StartConveyor.name}: Service-Call zum Starten des Bandes.")

        client = node.cli_conveyor
        if not client.service_is_ready():
            node.get_logger().warn(
                "cli_conveyor ist noch nicht bereit, versuche in tick() erneut."
            )
            self._future = None
            return

        req = SetBool.Request()
        req.data = True  # True = starten
        self._future = client.call_async(req)

    def tick(self, node:PickAndPlaceNode) -> str | None:
        if self._future is None:
            client = node.cli_conveyor
            if client.service_is_ready():
                node.get_logger().info("Service jetzt bereit, starte Start-Call.")
                req = SetBool.Request()
                req.data = True
                self._future = client.call_async(req)
            return None

        if not self._future.done():
            # Service läuft noch → im State bleiben
            return None

        result = self._future.result()
        if result is None:
            node.get_logger().error(
                "Service-Call conveyor/set_running (Start) ohne Ergebnis."
            )
            return "WAIT_FOR_OBJECT"

        if result.success:
            node.get_logger().info("Fließband erfolgreich gestartet. Neuer Zyklus beginnt.")
            node.conveyor_running = True

            # Kontext für den nächsten Durchlauf zurücksetzen
            node.object_detected = False
            node.vacuum_ok = False
            node.object_color = None
            node.object_pose = None

            return "WAIT_FOR_OBJECT"

        node.get_logger().error(f"Fehler beim Starten des Bandes: {result.message}")
        return "WAIT_FOR_OBJECT"

    def on_exit(self, node) -> None:
        node.get_logger().info(f"EXIT {StartConveyor.name}")
        self._future = None
