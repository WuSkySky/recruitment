import pytest

from recruitment_sim_player_web.protocol import (
    parse_input,
    referee_command_allowed,
    RoleRegistry,
    validate_role,
)


def test_parse_input_snapshot():
    snapshot = parse_input(
        {
            "sequence": 7,
            "active": True,
            "mouse_dx": 12,
            "mouse_dy": -3.5,
            "pressed_keys": ["KeyW", "ShiftLeft", "MouseLeft", "KeyW"],
        }
    )
    assert snapshot.sequence == 7
    assert snapshot.mouse_dx == 12.0
    assert snapshot.mouse_dy == -3.5
    assert snapshot.pressed_keys == ["KeyW", "MouseLeft", "ShiftLeft"]


@pytest.mark.parametrize(
    "field,value",
    [
        ("sequence", -1),
        ("sequence", 2**32),
        ("mouse_dx", float("nan")),
        ("active", 1),
        ("pressed_keys", "KeyW"),
        ("pressed_keys", [1]),
        ("pressed_keys", [""]),
        ("pressed_keys", None),
    ],
)
def test_rejects_invalid_input(field, value):
    payload = {"sequence": 1, "pressed_keys": [], field: value}
    with pytest.raises(ValueError):
        parse_input(payload)


@pytest.mark.parametrize("role", ["red", "blue", "referee"])
def test_accepts_all_web_roles(role):
    assert validate_role(role) == role


def test_rejects_unknown_web_role():
    with pytest.raises(ValueError):
        validate_role("spectator")


@pytest.mark.parametrize(
    "command,state,allowed",
    [
        ("reset", 0, True),
        ("reset", 2, False),
        ("reset", 4, True),
        ("reset", 5, True),
        ("reset", 6, False),
        ("start", 6, True),
        ("start", 2, False),
        ("end", 2, True),
        ("end", 6, False),
        ("invalid", 0, False),
    ],
)
def test_referee_command_rules(command, state, allowed):
    assert referee_command_allowed(command, state) is allowed


def test_role_registry_isolates_three_roles_and_rejects_duplicates():
    registry = RoleRegistry()
    red = object()
    blue = object()
    referee = object()
    assert registry.acquire("red", red)
    assert registry.acquire("blue", blue)
    assert registry.acquire("referee", referee)
    assert not registry.acquire("red", object())
    assert registry.occupied("red")
    assert registry.release("red", red)
    assert not registry.occupied("red")
    assert registry.occupied("blue")
    assert registry.occupied("referee")


def test_role_registry_only_owner_can_release_role():
    registry = RoleRegistry()
    owner = object()
    assert registry.acquire("blue", owner)
    assert not registry.release("blue", object())
    assert registry.occupied("blue")


def test_inactive_input_clears_keys_and_motion():
    snapshot = parse_input({
        "sequence": 1, "active": False, "pressed_keys": ["KeyW"],
        "mouse_dx": 10, "mouse_dy": -2,
    })
    assert snapshot.pressed_keys == []
    assert snapshot.mouse_dx == snapshot.mouse_dy == 0


def test_empty_input_and_extensible_key_names():
    assert parse_input({"sequence": 1, "pressed_keys": []}).pressed_keys == []
    assert parse_input({
        "sequence": 2, "active": True, "pressed_keys": ["CustomKey"],
    }).pressed_keys == ["CustomKey"]
