import math
from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class InputSnapshot:
    sequence: int
    active: bool
    mouse_dx: float
    mouse_dy: float
    key_w: bool
    key_a: bool
    key_s: bool
    key_d: bool
    left_button: bool
    right_button: bool


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

    return InputSnapshot(
        sequence=sequence,
        active=_boolean(payload, "active"),
        mouse_dx=float(dx),
        mouse_dy=float(dy),
        key_w=_boolean(payload, "key_w"),
        key_a=_boolean(payload, "key_a"),
        key_s=_boolean(payload, "key_s"),
        key_d=_boolean(payload, "key_d"),
        left_button=_boolean(payload, "left_button"),
        right_button=_boolean(payload, "right_button"),
    )
