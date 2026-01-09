from __future__ import annotations

from abc import ABC, abstractmethod


class State(ABC):
    """Abstrakte Basisklasse für alle Zustände der State-Machine."""

    def __init__(self, name: str):
        self.name = name

    def on_enter(self, node) -> None:
        """Eingangsaktion - wird einmalig beim Eintritt in den Zustand ausgeführt."""
        pass

    def on_exit(self, node) -> None:
        """Ausgangsaktion - wird einmalig beim Verlassen des Zustands ausgeführt."""
        pass

    @abstractmethod
    def tick(self, node) -> str | None:
        """
        Zustandsaktion + Übergangslogik.
        Wird zyklisch (im Timer) aufgerufen.

        Rückgabe:
            - None       → im aktuellen Zustand bleiben
            - "STATE_X"  → Übergang zu STATE_X
        """
        ...


class StateMachine:
    """Einfache State-Machine, die States verwaltet und Übergänge ausführt."""

    def __init__(self, node):
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
        """Ein Tick der State-Machine - z.B. aus einem ROS2-Timer aufgerufen."""
        if self._current_state is None:
            return

        next_name = self._current_state.tick(self._node)

        if next_name is not None and next_name != self._current_state.name:
            # Ausgangsaktion
            self._current_state.on_exit(self._node)

            old_name = self._current_state.name
            self._current_state = self._states[next_name]
            self._node.get_logger().info(
                f"Zustandswechsel: {old_name} → {next_name}"
            )

            # Eingangsaktion
            self._current_state.on_enter(self._node)
