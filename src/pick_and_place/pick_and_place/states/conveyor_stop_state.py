from __future__ import annotations
from typing import TYPE_CHECKING

import rclpy
from std_srvs.srv import SetBool
from rclpy import Future
from pick_and_place.state_machine import State

import time

if TYPE_CHECKING:
    # wird nur von mypy / VS Code etc. verwendet, nicht zur Laufzeit importiert
    from pick_and_place.pick_and_place_node import PickAndPlaceNode

class StopConveyor(State):
    """
    Stoppt das Fließband über einen asynchronen Service-Call.
    Demonstriert Future + Polling im tick().
    """
    name:str="STOP_CONVEYOR"
    timeout:float=5.0
    def __init__(self) -> None:
        super().__init__(StopConveyor.name)
        self._future: Future | None = None

    def _call_srv_stop_conveyor(self, node: PickAndPlaceNode) -> Future:
        client = node.cli_conveyor
        if not client.service_is_ready():
            node.get_logger().warn(
                "cli_conveyor ist noch nicht bereit, versuche es erneut..."
            )
            self._future = None
            return

        req = SetBool.Request()
        req.data = False  # False = stoppen
        self._future = client.call_async(req)

    def on_enter(self, node: PickAndPlaceNode) -> None:
        self.time_enter = time.time()
        node.get_logger().info(f"ENTER {self.name}: Service-Call zum Stoppen des Bandes.")
        self._call_srv_stop_conveyor(node)

    def tick(self, node: PickAndPlaceNode) -> str | None:
        if time.time() - self.time_enter > StopConveyor.timeout:
            node.get_logger().error(f"Abbruch wegen Timeout {self.name} -> WAIT_FOR_OBJECT")
            if self._future is not None:
                self._future.cancel()
                self._future=None
            return "WAIT_FOR_OBJECT"
        
        if self._future is None:
            self._call_srv_stop_conveyor(node)
            return None
        
        if not self._future.done():
            return None

        result : SetBool.Response = self._future.result()
        if result is None:
            node.get_logger().error(
                "Service-Call conveyor/set_running (Stop) lieferte kein Ergebnis."
            )
            return "WAIT_FOR_OBJECT"

        if result.success:
            node.get_logger().info("Fließband erfolgreich gestoppt.")
            node.conveyor_running = False
            return "ANALYZE_OBJECT"

        node.get_logger().error(f"Fehler beim Stoppen des Bandes: {result.message}")
        return "WAIT_FOR_OBJECT"

    def on_exit(self, node: PickAndPlaceNode) -> None:
        node.get_logger().info(f"{self.name}")
        self._future = None
