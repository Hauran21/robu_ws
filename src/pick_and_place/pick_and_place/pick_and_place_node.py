from __future__ import annotations
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from pick_and_place.state_machine import StateMachine

# States importieren
from pick_and_place.states.wait_for_object_state import WaitForObject
from pick_and_place.states.conveyor_stop_state import StopConveyor
from pick_and_place.states.analyse_object_state import AnalyzeObject
from pick_and_place.states.move_to_pick_state import MoveToPick
from pick_and_place.states.grip_object_state import GripObject
from pick_and_place.states.move_to_place_state import MoveToPlace
from pick_and_place.states.release_object_state import ReleaseObject
from pick_and_place.states.move_to_home_state import MoveToHome
from pick_and_place.states.conveyor_start_state import StartConveyor
from pick_and_place.states.idle_state import Idle

class PickAndPlaceNode(Node):
    """
    ROS2-Node, der die State-Machine über einen Timer ausführt.
    """
    
    def __init__(self) -> None:
        super().__init__("pick_and_place_sm_node")

        # „öffentliche“ Daten, die States lesen/schreiben
        # -> Merker für die States erzeugen 
        self.object_detected: bool = False
        self.vacuum_ok: bool = False
        self.conveyor_running: bool = True
        self.object_color: str | None = None
        self.object_pose: object | None = None  # hier später z.B. eine Pose eintragen

        # State-Machine erzeugen
        # Als Kontext für State-Machine wird unser Node übergeben
        # In unserer Klasse PickAndPlaceNode befinden sich alle Infos (Merker, Publisher, Service, ...) damit die State-Machine arbeiten kann
        # SAUBERER WÄRE ES wenn statt des kompleten Nodes (hier self) nur eine Schnittstelle (mit den notwendigen Merkern) übergeben wird
        # Thema Entspaghettisierung!!! -> Unsere Code Teile (z.B.: State-Machine unahängig von ROS) sind nich unabhängig voneinander testbarr -> 
        # das führt bei größteren Projekten zu Problemen
        self.sm = StateMachine(self)

        # States registrieren
        self.sm.add_state(Idle())
        self.sm.add_state(WaitForObject())
        self.sm.add_state(StopConveyor())
        self.sm.add_state(AnalyzeObject())
        self.sm.add_state(MoveToPick())
        self.sm.add_state(GripObject())
        self.sm.add_state(MoveToPlace())
        self.sm.add_state(ReleaseObject())
        self.sm.add_state(MoveToHome())
        self.sm.add_state(StartConveyor())

        # Service-Client für das Fließband
        self.cli_conveyor = self.create_client(SetBool, "conveyor/set_running")
        self.get_logger().info("Warte auf Service conveyor/set_running...")

        self.cli_conveyor.wait_for_service()
        self.get_logger().info("Service conveyor/set_running verfügbar.")

        # Subscriber (Lichtschranke, Vakuum)
        self.create_subscription(Bool, "light_barrier/detected", self._sub_light_barrier_cb, 10)
        self.create_subscription(Bool, "vacuum/grip_ok", self._sub_vacuum_cb, 10)

        # Timer für State-Machine
        self._timer = self.create_timer(0.2, self._timer_sm_cb)

        # Initialer Zustand
        self.sm.set_initial_state("WAIT_FOR_OBJECT")

    # --- Callbacks ---
    def _sub_light_barrier_cb(self, msg: Bool) -> None:
        # In einem zweiten Programm läuft ein Publisher. Dieser prüft die Werte von der Lichtschranke.
        # Wenn sich der Zustand der Lichtschranke ändert, sendet der Publisher True 
        # (Objekt blockiert die Lichtschranke) oder False.
        
        #Wir merken uns nun hier den Zustand der Lichtschranke für den WaitForObjectState
        
        self.object_detected = msg.data

    def _sub_vacuum_cb(self, msg: Bool) -> None:
        self.vacuum_ok = msg.data

    def _timer_sm_cb(self) -> None:
        self.sm.step()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
