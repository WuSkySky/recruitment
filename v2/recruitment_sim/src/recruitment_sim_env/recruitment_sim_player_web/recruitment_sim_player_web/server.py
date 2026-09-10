import asyncio
from collections import deque
from dataclasses import asdict, dataclass
import json
import os
import secrets
import threading
import time
from typing import Dict, List, Optional, Set

from aiohttp import WSMsgType, web
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
    VideoStreamTrack,
)
from ament_index_python.packages import get_package_share_directory
from av import VideoFrame
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from recruitment_sim_interfaces.msg import (
    MatchStatus,
    PlayerInput,
    RobotStatus,
)
from recruitment_sim_interfaces.srv import ControlMatch
from sensor_msgs.msg import Image
import yaml

from .protocol import (
    ALL_ROLES,
    PLAYER_ROLES,
    InputSnapshot,
    RoleRegistry,
    parse_input,
    referee_command_allowed,
    validate_role,
)


ROBOT_TYPES = {
    "pb2025_infantry_robot": "infantry",
    "pb2025_sentry_robot": "sentry",
}
MATCH_STATUS_TIMEOUT = 0.5
CONTROL_TIMEOUT = 2.0
CONTROL_COMMANDS = {
    "start": ControlMatch.Request.START,
    "end": ControlMatch.Request.END,
    "reset": ControlMatch.Request.RESUME,
}


@dataclass(frozen=True)
class RobotDescriptor:
    name: str
    team: str
    kind: str
    namespace: str


def load_robots(path: str) -> List[RobotDescriptor]:
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if (
        not isinstance(config, dict)
        or not isinstance(config.get("robots"), list)
    ):
        raise RuntimeError("robots_file must contain a robots list")

    robots = []
    for entry in config["robots"]:
        try:
            kind = ROBOT_TYPES[entry["type"]]
            name = str(entry["name"])
            team = str(entry["color"])
        except (KeyError, TypeError) as exc:
            raise RuntimeError(
                f"invalid robot entry in {path}: {entry!r}"
            ) from exc
        robots.append(RobotDescriptor(name, team, kind, f"/{team}/{kind}"))
    return robots


class LatestImage:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._message: Optional[Image] = None
        self._sequence = 0
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._event: Optional[asyncio.Event] = None
        self._arrivals = deque(maxlen=120)

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        with self._lock:
            self._loop = loop
            self._event = asyncio.Event()

    def push(self, message: Image) -> None:
        now = time.monotonic()
        with self._lock:
            self._message = message
            self._sequence += 1
            self._arrivals.append(now)
            loop = self._loop
            event = self._event
        if loop is not None and event is not None:
            loop.call_soon_threadsafe(event.set)

    async def next(self, after_sequence: int):
        while True:
            with self._lock:
                event = self._event
            if event is None:
                await asyncio.sleep(0.01)
                continue
            event.clear()
            with self._lock:
                if (
                    self._message is not None
                    and self._sequence != after_sequence
                ):
                    return self._sequence, self._message
            await event.wait()

    def receive_fps(self) -> float:
        with self._lock:
            arrivals = list(self._arrivals)
        if len(arrivals) < 2:
            return 0.0
        window_start = max(arrivals[0], arrivals[-1] - 2.0)
        samples = [stamp for stamp in arrivals if stamp >= window_start]
        duration = samples[-1] - samples[0]
        return 0.0 if duration <= 0 else (len(samples) - 1) / duration


class RosVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, latest: LatestImage) -> None:
        super().__init__()
        self._latest = latest
        self._sequence = 0

    async def recv(self) -> VideoFrame:
        self._sequence, message = await self._latest.next(self._sequence)
        frame = ros_image_to_video_frame(message)
        pts, time_base = await self.next_timestamp()
        frame.pts = pts
        frame.time_base = time_base
        return frame


def ros_image_to_video_frame(message: Image) -> VideoFrame:
    formats = {
        "rgb8": (3, "rgb24"),
        "bgr8": (3, "bgr24"),
        "rgba8": (4, "rgba"),
        "bgra8": (4, "bgra"),
        "mono8": (1, "gray"),
    }
    if message.encoding not in formats:
        raise ValueError(f"unsupported camera encoding: {message.encoding}")
    channels, frame_format = formats[message.encoding]
    required = int(message.height) * int(message.step)
    buffer = np.frombuffer(message.data, dtype=np.uint8)
    if buffer.size < required:
        raise ValueError("camera image buffer is shorter than height * step")
    rows = buffer[:required].reshape((message.height, message.step))
    pixels = rows[:, : message.width * channels]
    if channels == 1:
        pixels = pixels.reshape((message.height, message.width))
    else:
        pixels = pixels.reshape((message.height, message.width, channels))
    return VideoFrame.from_ndarray(
        np.ascontiguousarray(pixels), format=frame_format
    )


class CompetitionWebNode(Node):
    def __init__(self) -> None:
        super().__init__("player_web")
        self.declare_parameter("robots_file", "")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8080)

        robots_file = self.get_parameter("robots_file").value
        if not robots_file:
            raise RuntimeError(
                "robots_file parameter is required"
            )
        self.robots = load_robots(robots_file)
        self.player_robots: Dict[str, RobotDescriptor] = {}
        for team in sorted(PLAYER_ROLES):
            selected = [
                robot
                for robot in self.robots
                if robot.team == team and robot.kind == "infantry"
            ]
            if len(selected) != 1:
                raise RuntimeError(
                    "robots_file must contain exactly one "
                    f"{team} infantry robot"
                )
            self.player_robots[team] = selected[0]

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.latest_images = {team: LatestImage() for team in PLAYER_ROLES}
        self._input_publishers = {}
        self._image_subscriptions = []
        for team, robot in self.player_robots.items():
            self._input_publishers[team] = self.create_publisher(
                PlayerInput, f"{robot.namespace}/player_input", sensor_qos
            )
            self._image_subscriptions.append(
                self.create_subscription(
                    Image,
                    f"{robot.namespace}/camera/image",
                    self.latest_images[team].push,
                    sensor_qos,
                )
            )

        self._state_lock = threading.Lock()
        self._match: Optional[MatchStatus] = None
        self._match_received_at = 0.0
        self._robot_states: Dict[str, RobotStatus] = {}
        self._robot_received_at: Dict[str, float] = {}
        self._match_subscription = self.create_subscription(
            MatchStatus,
            "/referee_system/match/status",
            self._on_match,
            state_qos,
        )
        self._robot_subscriptions = []
        for robot in self.robots:
            self._robot_subscriptions.append(
                self.create_subscription(
                    RobotStatus,
                    f"/referee_system/{robot.name}/status",
                    lambda message, name=robot.name: self._on_robot(
                        name, message
                    ),
                    state_qos,
                )
            )
        self._control_client = self.create_client(
            ControlMatch, "/referee_system/match/control"
        )
        self.get_logger().info(
            "competition web serves red player, blue player, and referee roles"
        )

    def _on_match(self, message: MatchStatus) -> None:
        with self._state_lock:
            self._match = message
            self._match_received_at = time.monotonic()

    def _on_robot(self, name: str, message: RobotStatus) -> None:
        with self._state_lock:
            self._robot_states[name] = message
            self._robot_received_at[name] = time.monotonic()

    def match_online(self) -> bool:
        with self._state_lock:
            received_at = self._match_received_at
        return (
            received_at > 0.0
            and time.monotonic() - received_at <= MATCH_STATUS_TIMEOUT
        )

    def current_match_state(self) -> Optional[int]:
        with self._state_lock:
            match = self._match
            received_at = self._match_received_at
        if (
            match is None
            or time.monotonic() - received_at > MATCH_STATUS_TIMEOUT
        ):
            return None
        return int(match.state)

    def publish_input(self, team: str, snapshot: InputSnapshot) -> None:
        if team not in PLAYER_ROLES:
            raise ValueError("input role must be red or blue")
        message = PlayerInput()
        message.header.stamp = self.get_clock().now().to_msg()
        message.sequence = snapshot.sequence
        message.active = snapshot.active
        if snapshot.active:
            message.mouse_dx = snapshot.mouse_dx
            message.mouse_dy = snapshot.mouse_dy
            message.pressed_keys = snapshot.pressed_keys
        self._input_publishers[team].publish(message)

    def publish_neutral(self, team: str, sequence: int = 0) -> None:
        self.publish_input(
            team,
            InputSnapshot(
                sequence, False, 0.0, 0.0,
                [],
            ),
        )

    def status_payload(self, role: str) -> dict:
        now = time.monotonic()
        with self._state_lock:
            match = self._match
            match_at = self._match_received_at
            states = dict(self._robot_states)
            state_times = dict(self._robot_received_at)
        match_available = (
            match is not None
            and now - match_at <= MATCH_STATUS_TIMEOUT
        )
        robots = []
        for descriptor in self.robots:
            state = states.get(descriptor.name)
            available = (
                state is not None
                and now - state_times.get(descriptor.name, 0)
                <= MATCH_STATUS_TIMEOUT
            )
            robots.append(
                {
                    **asdict(descriptor),
                    "max_hp": int(state.max_hp) if state else 0,
                    "current_hp": int(state.current_hp) if state else 0,
                    "shooter_heat": (
                        float(state.shooter_heat) if state else 0.0
                    ),
                    "heat_limit": float(state.heat_limit) if state else 0.0,
                    "alive": bool(state.alive) if state else False,
                    "available": available,
                }
            )
        payload = {
            "type": "status",
            "role": role,
            "match": {
                "available": match_available,
                "state": int(match.state) if match else 0,
                "result": int(match.result) if match else 0,
                "remaining_seconds": (
                    max(0.0, float(match.remaining_seconds)) if match else 0.0
                ),
                "red_victory_points": (
                    int(match.red_victory_points) if match else 0
                ),
                "blue_victory_points": (
                    int(match.blue_victory_points) if match else 0
                ),
                "control_zone_owner": (
                    str(match.control_zone_owner) if match else ""
                ),
                "end_reason": str(match.end_reason) if match else "",
                "error_message": str(match.error_message) if match else "",
            },
            "robots": robots,
        }
        if role in PLAYER_ROLES:
            robot = self.player_robots[role]
            payload.update(
                {
                    "player_team": role,
                    "player_robot": robot.name,
                    "ros_receive_fps": round(
                        self.latest_images[role].receive_fps(), 1
                    ),
                }
            )
        return payload

    async def control_match(self, command: int):
        if not self._control_client.service_is_ready():
            raise RuntimeError("裁判控制服务不可用")
        request = ControlMatch.Request()
        request.command = command
        ros_future = self._control_client.call_async(request)
        loop = asyncio.get_running_loop()
        bridge = loop.create_future()

        def completed(future) -> None:
            try:
                response = future.result()
                loop.call_soon_threadsafe(
                    lambda value=response: (
                        None if bridge.done() else bridge.set_result(value)
                    )
                )
            # Forward ROS failure to the web event loop.
            except Exception as exc:
                loop.call_soon_threadsafe(
                    lambda error=exc: (
                        None if bridge.done() else bridge.set_exception(error)
                    )
                )

        ros_future.add_done_callback(completed)
        try:
            return await asyncio.wait_for(bridge, timeout=CONTROL_TIMEOUT)
        except asyncio.TimeoutError:
            self._control_client.remove_pending_request(ros_future)
            raise RuntimeError("裁判控制请求超时") from None


class CompetitionWebServer:
    def __init__(self, node: CompetitionWebNode) -> None:
        self.node = node
        self.controllers: Dict[str, web.WebSocketResponse] = {}
        self.role_registry = RoleRegistry()
        self.tokens: Dict[str, str] = {}
        self.peer_connections: Dict[str, Set[RTCPeerConnection]] = {}
        self.status_task: Optional[asyncio.Task] = None
        self.referee_request_pending = False
        self._session_lock = asyncio.Lock()
        share = get_package_share_directory("recruitment_sim_player_web")
        self.web_root = os.path.join(
            share, "recruitment_sim_player_web", "web"
        )

    def application(self) -> web.Application:
        application = web.Application()
        application.router.add_get("/", self.index)
        application.router.add_get("/player/red", self.index)
        application.router.add_get("/player/blue", self.index)
        application.router.add_get("/referee", self.index)
        application.router.add_get("/api/roles", self.roles)
        application.router.add_get("/ws/{role}", self.websocket)
        application.router.add_post("/api/webrtc/offer", self.offer)
        application.router.add_static(
            "/assets",
            os.path.join(self.web_root, "assets"),
            follow_symlinks=True,
        )
        application.on_startup.append(self.on_startup)
        application.on_cleanup.append(self.on_cleanup)
        return application

    async def index(self, _request: web.Request) -> web.Response:
        return web.FileResponse(os.path.join(self.web_root, "index.html"))

    async def roles(self, _request: web.Request) -> web.Response:
        async with self._session_lock:
            occupied = {
                role: self.role_registry.occupied(role)
                for role in ALL_ROLES
            }
        return web.json_response(
            {
                "roles": {
                    role: {"occupied": occupied[role], "available": True}
                    for role in sorted(ALL_ROLES)
                },
                "referee_online": self.node.match_online(),
            }
        )

    async def on_startup(self, _application: web.Application) -> None:
        loop = asyncio.get_running_loop()
        for image in self.node.latest_images.values():
            image.attach_loop(loop)
        self.status_task = asyncio.create_task(self.send_status_loop())

    async def on_cleanup(self, _application: web.Application) -> None:
        if self.status_task is not None:
            self.status_task.cancel()
            await asyncio.gather(self.status_task, return_exceptions=True)
        if rclpy.ok():
            for team in PLAYER_ROLES:
                self.node.publish_neutral(team)
        for socket in list(self.controllers.values()):
            await socket.close()
        peers = [
            peer
            for group in self.peer_connections.values()
            for peer in group
        ]
        await asyncio.gather(
            *(peer.close() for peer in peers), return_exceptions=True
        )

    async def websocket(self, request: web.Request) -> web.WebSocketResponse:
        try:
            role = validate_role(request.match_info["role"])
        except ValueError:
            raise web.HTTPNotFound(text="unknown role")
        socket = web.WebSocketResponse(heartbeat=5.0)
        await socket.prepare(request)
        token = await self.claim_role(role, socket)
        if token is None:
            await socket.send_json({"type": "occupied", "role": role})
            await socket.close()
            return socket
        ready = {"type": "ready", "role": role, "token": token}
        if role in PLAYER_ROLES:
            ready.update(
                {
                    "player_team": role,
                    "player_robot": self.node.player_robots[role].name,
                }
            )
        try:
            await socket.send_json(ready)
            async for message in socket:
                if message.type != WSMsgType.TEXT:
                    continue
                try:
                    payload = json.loads(message.data)
                    kind = payload.get("type")
                    if kind == "input":
                        if role not in PLAYER_ROLES:
                            raise ValueError(
                                "referee role cannot publish player input"
                            )
                        self.node.publish_input(role, parse_input(payload))
                    elif kind == "referee_command":
                        if role != "referee":
                            raise ValueError(
                                "player role cannot control the match"
                            )
                        await self.handle_referee_command(socket, payload)
                    elif kind == "ping":
                        await socket.send_json(
                            {"type": "pong", "sent_at": payload.get("sent_at")}
                        )
                except (ValueError, TypeError, json.JSONDecodeError) as exc:
                    await socket.send_json(
                        {"type": "error", "message": str(exc)}
                    )
        finally:
            await self.release_role(role, socket, token)
        return socket

    async def claim_role(
        self, role: str, socket: web.WebSocketResponse
    ) -> Optional[str]:
        async with self._session_lock:
            if not self.role_registry.acquire(role, socket):
                return None
            token = secrets.token_urlsafe(24)
            self.controllers[role] = socket
            self.tokens[role] = token
            self.peer_connections[token] = set()
            return token

    async def release_role(
        self,
        role: str,
        socket: web.WebSocketResponse,
        token: str,
    ) -> None:
        async with self._session_lock:
            if self.controllers.get(role) is not socket:
                return
            self.controllers.pop(role, None)
            self.role_registry.release(role, socket)
            self.tokens.pop(role, None)
            peers = list(self.peer_connections.pop(token, set()))
        if role in PLAYER_ROLES and rclpy.ok():
            self.node.publish_neutral(role)
        await asyncio.gather(
            *(peer.close() for peer in peers), return_exceptions=True
        )

    async def handle_referee_command(self, socket, payload: dict) -> None:
        request_id = payload.get("request_id")
        command_name = payload.get("command")
        if not isinstance(request_id, int) or isinstance(request_id, bool):
            raise ValueError("request_id must be an integer")
        if command_name not in CONTROL_COMMANDS:
            raise ValueError("unknown referee command")
        if self.referee_request_pending:
            await self.send_referee_result(
                socket, request_id, False, "已有裁判控制请求正在执行"
            )
            return
        state = self.node.current_match_state()
        if state is None:
            await self.send_referee_result(socket, request_id, False, "裁判节点离线")
            return
        if not referee_command_allowed(command_name, state):
            await self.send_referee_result(
                socket, request_id, False, "当前比赛状态不允许该操作"
            )
            return
        self.referee_request_pending = True
        try:
            response = await self.node.control_match(
                CONTROL_COMMANDS[command_name]
            )
            await self.send_referee_result(
                socket,
                request_id,
                bool(response.accepted),
                str(response.message),
            )
        # Keep the referee page alive on ROS client failures.
        except Exception as exc:
            await self.send_referee_result(socket, request_id, False, str(exc))
        finally:
            self.referee_request_pending = False

    @staticmethod
    async def send_referee_result(
        socket, request_id, accepted, message
    ) -> None:
        await socket.send_json(
            {
                "type": "referee_result",
                "request_id": request_id,
                "accepted": accepted,
                "message": message,
            }
        )

    async def offer(self, request: web.Request) -> web.Response:
        payload = await request.json()
        role = payload.get("role")
        token = payload.get("token")
        peers = self.peer_connections.get(token)
        if (
            role not in PLAYER_ROLES
            or self.tokens.get(role) != token
            or peers is None
        ):
            raise web.HTTPForbidden(text="no active player session")
        peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        peers.add(peer)
        transceiver = peer.addTransceiver(
            RosVideoTrack(self.node.latest_images[role]), direction="sendonly"
        )
        codecs = [
            codec
            for codec in RTCRtpSender.getCapabilities("video").codecs
            if codec.mimeType.lower() in {"video/vp8", "video/rtx"}
        ]
        if codecs:
            transceiver.setCodecPreferences(codecs)

        @peer.on("connectionstatechange")
        async def connection_state_changed() -> None:
            if peer.connectionState in {"failed", "closed"}:
                await peer.close()
                self.peer_connections.get(token, set()).discard(peer)

        offer = RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])
        await peer.setRemoteDescription(offer)
        answer = await peer.createAnswer()
        await peer.setLocalDescription(answer)
        return web.json_response(
            {
                "sdp": peer.localDescription.sdp,
                "type": peer.localDescription.type,
            }
        )

    async def send_status_loop(self) -> None:
        while True:
            await asyncio.sleep(0.1)
            for role, socket in list(self.controllers.items()):
                if socket.closed:
                    continue
                try:
                    payload = self.node.status_payload(role)
                    if role == "referee":
                        payload["request_pending"] = (
                            self.referee_request_pending
                        )
                    await socket.send_json(payload)
                except (ConnectionError, RuntimeError):
                    pass


async def run_server(node: CompetitionWebNode) -> None:
    server = CompetitionWebServer(node)
    runner = web.AppRunner(server.application())
    await runner.setup()
    host = node.get_parameter("bind_address").value
    port = int(node.get_parameter("port").value)
    site = web.TCPSite(runner, host=host, port=port)
    await site.start()
    node.get_logger().info(
        f"competition web server listening on http://{host}:{port}"
    )
    try:
        await asyncio.Event().wait()
    finally:
        await runner.cleanup()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CompetitionWebNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    try:
        asyncio.run(run_server(node))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
