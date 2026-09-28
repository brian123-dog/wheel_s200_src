from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class VerificationRequest:
    sensor: str
    expected: Any
    timeout_s: float = 1.0


@dataclass(frozen=True)
class VerificationResult:
    sensor: str
    expected: Any
    observed: Any
    passed: bool


class StateVerifier(ABC):
    """Interface for task-board or cabinet-state verification."""

    @abstractmethod
    def verify(self, request: VerificationRequest) -> VerificationResult:
        raise NotImplementedError


class MockStateVerifier(StateVerifier):
    def __init__(self, states: dict[str, Any] | None = None) -> None:
        self.states = dict(states or {})

    def set_state(self, sensor: str, value: Any) -> None:
        self.states[sensor] = value

    def verify(self, request: VerificationRequest) -> VerificationResult:
        observed = self.states.get(request.sensor)
        return VerificationResult(
            sensor=request.sensor,
            expected=request.expected,
            observed=observed,
            passed=observed == request.expected,
        )
