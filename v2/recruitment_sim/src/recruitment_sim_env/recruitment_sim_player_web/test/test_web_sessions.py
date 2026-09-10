import asyncio
from pathlib import Path
from types import SimpleNamespace

from recruitment_sim_player_web import server as server_module
from recruitment_sim_player_web.server import (
    CompetitionWebServer,
    RobotDescriptor,
)


class FakeImage:
    def attach_loop(self, _loop):
        pass


class FakeNode:
    def __init__(self):
        self.latest_images = {
            "red": FakeImage(),
            "blue": FakeImage(),
        }
        self.player_robots = {
            "red": RobotDescriptor(
                "red_infantry_robot",
                "red",
                "infantry",
                "/red/infantry",
            ),
            "blue": RobotDescriptor(
                "blue_infantry_robot",
                "blue",
                "infantry",
                "/blue/infantry",
            ),
        }
        self.neutralized = []
        self.current_state = 0
        self.control_calls = []
        self.control_error = None

    def publish_neutral(self, team, _sequence=0):
        self.neutralized.append(team)

    def current_match_state(self):
        return self.current_state

    async def control_match(self, command):
        self.control_calls.append(command)
        if self.control_error is not None:
            raise self.control_error
        return SimpleNamespace(accepted=True, message="accepted")


class FakeSocket:
    def __init__(self):
        self.messages = []

    async def send_json(self, message):
        self.messages.append(message)


def make_server(monkeypatch):
    package_root = Path(__file__).parents[1]
    monkeypatch.setattr(
        server_module,
        "get_package_share_directory",
        lambda _package: str(package_root),
    )
    node = FakeNode()
    return node, CompetitionWebServer(node)


def test_three_roles_are_isolated(monkeypatch):
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)
    node, web_server = make_server(monkeypatch)

    async def scenario():
        red = object()
        blue = object()
        referee = object()
        duplicate = object()

        red_token = await web_server.claim_role("red", red)
        blue_token = await web_server.claim_role("blue", blue)
        referee_token = await web_server.claim_role("referee", referee)
        assert red_token is not None
        assert blue_token is not None
        assert referee_token is not None
        assert await web_server.claim_role("red", duplicate) is None
        assert web_server.role_registry.occupied("red")
        assert web_server.role_registry.occupied("blue")
        assert web_server.role_registry.occupied("referee")

        await web_server.release_role("red", red, red_token)
        assert not web_server.role_registry.occupied("red")
        assert web_server.role_registry.occupied("blue")
        assert web_server.role_registry.occupied("referee")
        assert node.neutralized == ["red"]

        await web_server.release_role("blue", blue, blue_token)
        await web_server.release_role(
            "referee", referee, referee_token
        )

    asyncio.run(scenario())


def test_release_ack_is_sent_after_role_is_free(monkeypatch):
    monkeypatch.setattr(server_module.rclpy, "ok", lambda: True)
    node, web_server = make_server(monkeypatch)

    class ReleaseSocket(FakeSocket):
        async def send_json(self, message):
            assert not web_server.role_registry.occupied("red")
            await super().send_json(message)

    socket = ReleaseSocket()

    async def scenario():
        token = await web_server.claim_role("red", socket)
        assert token is not None
        await web_server.handle_release("red", socket, token)
        assert socket.messages == [{"type": "released", "role": "red"}]
        assert node.neutralized == ["red"]

    asyncio.run(scenario())


def test_referee_command_state_guard_and_result(monkeypatch):
    node, web_server = make_server(monkeypatch)
    socket = FakeSocket()

    async def scenario():
        node.current_state = 2
        await web_server.handle_referee_command(
            socket,
            {"request_id": 1, "command": "reset"},
        )
        assert socket.messages[-1]["accepted"] is False
        assert node.control_calls == []

        node.current_state = 6
        await web_server.handle_referee_command(
            socket,
            {"request_id": 2, "command": "start"},
        )
        assert socket.messages[-1] == {
            "type": "referee_result",
            "request_id": 2,
            "accepted": True,
            "message": "accepted",
        }
        assert len(node.control_calls) == 1

        node.current_state = 2
        node.control_error = RuntimeError("service unavailable")
        await web_server.handle_referee_command(
            socket,
            {"request_id": 3, "command": "end"},
        )
        assert socket.messages[-1]["accepted"] is False
        assert socket.messages[-1]["message"] == "service unavailable"

    asyncio.run(scenario())
