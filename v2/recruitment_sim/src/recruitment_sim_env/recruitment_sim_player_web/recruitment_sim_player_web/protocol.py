import math
from dataclasses import dataclass
from typing import Any, Mapping


PLAYER_ROLES = {"red", "blue"}
ALL_ROLES = PLAYER_ROLES | {"referee"}
REFEREE_ALLOWED_STATES = {
    "start": {6},
    "end": {2},
    "reset": {0, 4, 5},
}


@dataclass(frozen=True)
class InputSnapshot:
    sequence: int
    active: bool
    mouse_dx: float
    mouse_dy: float
    pressed_keys: list[str]


class RoleRegistry:
    def __init__(self) -> None:
        self._owners = {}

    def acquire(self, role: str, owner: Any) -> bool:
        validate_role(role)
        if role in self._owners:
            return False
        self._owners[role] = owner
        return True

    def release(self, role: str, owner: Any) -> bool:
        if self._owners.get(role) is not owner:
            return False
        del self._owners[role]
        return True

    def occupied(self, role: str) -> bool:
        validate_role(role)
        return role in self._owners


def _boolean(payload: Mapping[str, Any], name: str) -> bool:
    value = payload.get(name, False)
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a boolean")
    return value


def parse_input(payload: Mapping[str, Any]) -> InputSnapshot:
    sequence = payload.get("sequence")
    if (
        not isinstance(sequence, int)
        or isinstance(sequence, bool)
        or not 0 <= sequence <= 0xFFFFFFFF
    ):
        raise ValueError("sequence must be an unsigned 32-bit integer")

    dx = payload.get("mouse_dx", 0.0)
    dy = payload.get("mouse_dy", 0.0)
    if (
        isinstance(dx, bool)
        or not isinstance(dx, (int, float))
        or not math.isfinite(dx)
    ):
        raise ValueError("mouse_dx must be finite")
    if (
        isinstance(dy, bool)
        or not isinstance(dy, (int, float))
        or not math.isfinite(dy)
    ):
        raise ValueError("mouse_dy must be finite")

    keys = payload.get("pressed_keys")
    if not isinstance(keys, list) or any(
        not isinstance(code, str) or not code.strip() for code in keys
    ):
        raise ValueError("pressed_keys must be an array of non-empty strings")
    active = _boolean(payload, "active")

    return InputSnapshot(
        sequence=sequence,
        active=active,
        mouse_dx=float(dx) if active else 0.0,
        mouse_dy=float(dy) if active else 0.0,
        pressed_keys=sorted(set(keys)) if active else [],
    )


def validate_role(role: str) -> str:
    if role not in ALL_ROLES:
        raise ValueError("unknown web role")
    return role


def referee_command_allowed(command: str, state: int) -> bool:
    return (
        command in REFEREE_ALLOWED_STATES
        and state in REFEREE_ALLOWED_STATES[command]
    )
