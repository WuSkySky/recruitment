import pytest

from recruitment_sim_player_web.protocol import parse_input


def test_parse_input_snapshot():
    snapshot = parse_input(
        {
            "sequence": 7,
            "active": True,
            "mouse_dx": 12,
            "mouse_dy": -3.5,
            "key_w": True,
            "key_a": False,
            "key_s": False,
            "key_d": True,
            "left_button": True,
            "right_button": False,
        }
    )
    assert snapshot.sequence == 7
    assert snapshot.mouse_dx == 12.0
    assert snapshot.mouse_dy == -3.5
    assert snapshot.key_w and snapshot.key_d and snapshot.left_button


@pytest.mark.parametrize(
    "field,value",
    [
        ("sequence", -1),
        ("sequence", 2**32),
        ("mouse_dx", float("nan")),
        ("key_w", 1),
    ],
)
def test_rejects_invalid_input(field, value):
    payload = {"sequence": 1, field: value}
    with pytest.raises(ValueError):
        parse_input(payload)
