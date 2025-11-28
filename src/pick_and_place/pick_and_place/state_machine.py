from __future__ import annotations
from abc import ABC, abstractmethod


class State(ABC): # von der klasse abc gegerbt abstract base class (damit @abstractmethod funktioniert)
    """Basisklasse für alle Zustände der State-Machine."""

    def __init__(self, name: str) -> None:
        self.name = name

    def on_enter(self, node) -> None:
        """Wird einmalig beim Eintritt in den Zustand ausgeführt."""
        pass

    def on_exit(self, node) -> None:    
        """Wird einmalig beim Verlassen des Zustands ausgeführt."""
        pass

    @abstractmethod # tick muss später überschrieben werden -> wenn man einen state programmiert muss man eine tick funktion erstellen
    def tick(self, node) -> str | None:
        """
        Wird zyklisch aufgerufen.
        Muss entweder None (im Zustand bleiben)
        oder den Namen des nächsten States zurückgeben.
        """
        ...


class StateMachine:
    """Verwalten der States + Ablaufsteuerung."""

    def __init__(self, node) -> None:
        self._node = node
        self._states: dict[str, State] = {}
        self._current_state: State | None = None

    def add_state(self, state: State) -> None:
        self._states[state.name] = state

    def set_initial_state(self, name: str) -> None:
        self._current_state = self._states[name]
        self._node.get_logger().info(f"Initialer Zustand: {name}")
        self._current_state.on_enter(self._node)

    def step(self) -> None:
        """Soll aus dem ROS2-Timer aufgerufen werden."""
        if self._current_state is None:
            return

        next_name = self._current_state.tick(self._node)

        if next_name is not None and next_name != self._current_state.name:
            # Ausgangsaktion
            self._current_state.on_exit(self._node)

            old = self._current_state.name
            self._current_state = self._states[next_name]

            self._node.get_logger().info(
                f"Zustandswechsel: {old} → {next_name}"
            )

            # Eingangsaktion
            self._current_state.on_enter(self._node)