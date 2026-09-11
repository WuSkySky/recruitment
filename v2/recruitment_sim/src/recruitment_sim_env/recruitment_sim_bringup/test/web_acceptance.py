#!/usr/bin/env python3
"""End-to-end probe for the Web match terminal against a running bringup.

Covers: static pages, role registry, WebSocket keyboard/mouse input reaching
the robot domain, ping/pong, role exclusivity, WebRTC video offer, and the
referee reset/start control path.

The camera only produces frames while the simulation is running, so the probe
first resumes the match through the referee role when it finds the simulation
paused (FINISHED/ERROR/RESETTING/ENDING).
"""
import argparse
import asyncio
import json
import threading
import time

import aiohttp
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from recruitment_sim_interfaces.msg import PlayerInput
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration

RUNNING, FINISHED, ERROR, READY = 2, 4, 5, 6
RESETTABLE = {0, FINISHED, ERROR}
TRANSIENT = {1, 3, 7}  # STARTING, ENDING, RESETTING
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print(f"{'PASS' if ok else 'FAIL'} {name}" + (f" | {detail}" if detail else ""), flush=True)


class RosProbe:
    def __init__(self, domain=20, topic='/red/infantry/player_input'):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = Node('web_acceptance_probe', context=self.context)
        self.messages = []
        self.node.create_subscription(PlayerInput, topic, self.messages.append, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def wait_for(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


async def recv_until(socket, kind, timeout=8.0, buffer=None):
    """Read JSON frames until one has the requested type; buffer the rest."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f'no {kind} frame')
        message = await asyncio.wait_for(socket.receive(), timeout=remaining)
        if message.type != aiohttp.WSMsgType.TEXT:
            continue
        payload = json.loads(message.data)
        if payload.get('type') == kind:
            return payload
        if buffer is not None:
            buffer.append(payload)


async def recv_first(socket, kinds, timeout=8.0):
    """Read JSON frames until one of `kinds` arrives."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f'no {" or ".join(sorted(kinds))} frame')
        message = await asyncio.wait_for(socket.receive(), timeout=remaining)
        if message.type != aiohttp.WSMsgType.TEXT:
            continue
        payload = json.loads(message.data)
        if payload.get('type') in kinds:
            return payload


async def next_status(socket, timeout=8.0):
    payload = await recv_until(socket, 'status', timeout)
    return int(payload['match']['state'])


async def drain_while(socket, task):
    """Keep the socket readable while waiting for an asyncio task."""
    while not task.done():
        try:
            await asyncio.wait_for(socket.receive(), timeout=0.2)
        except asyncio.TimeoutError:
            pass
    return await task


async def read_video_frame(peer, timeout=15.0):
    track = await asyncio.wait_for(_wait_track(peer), timeout)
    return await asyncio.wait_for(track.recv(), timeout)


async def drive_to_ready(referee, state):
    """Use the referee page to bring the match to READY for live media.

    The page only offers reset from TRAINING/FINISHED/ERROR and end from
    RUNNING, so follow that state machine instead of forcing a reset.
    """
    deadline = time.monotonic() + 20
    while state in TRANSIENT and time.monotonic() < deadline:
        state = await next_status(referee)
    if state == READY:
        return True, 'already READY'
    if state == RUNNING:
        await referee.send_json({'type': 'referee_command', 'request_id': 50,
                                 'command': 'end'})
        result = await recv_until(referee, 'referee_result', timeout=15)
        if not result.get('accepted'):
            return False, 'end rejected: ' + str(result.get('message'))
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and state != FINISHED:
            state = await next_status(referee)
    if state not in RESETTABLE:
        return False, f'cannot reset from state {state}'
    await referee.send_json({'type': 'referee_command', 'request_id': 51,
                             'command': 'reset'})
    result = await recv_until(referee, 'referee_result', timeout=15)
    if not result.get('accepted'):
        return False, 'reset rejected: ' + str(result.get('message'))
    deadline = time.monotonic() + 40
    while time.monotonic() < deadline and state != READY:
        state = await next_status(referee)
    return state == READY, f'state={state}'


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--timeout', type=float, default=8.0)
    args = parser.parse_args()
    base = f'http://127.0.0.1:{args.port}'
    probe = RosProbe()
    try:
        async with aiohttp.ClientSession() as session:
            # --- static pages -------------------------------------------------
            for path, marker in (('/', 'html'), ('/referee', 'html'),
                                 ('/player/red', 'html'), ('/player/blue', 'html')):
                async with session.get(base + path) as response:
                    body = await response.text()
                    check(f'GET {path}', response.status == 200 and marker in body,
                          f'status={response.status} bytes={len(body)}')
            async with session.get(base + '/api/roles') as response:
                roles = await response.json()
                names = set(roles.get('roles', {}))
                check('GET /api/roles', response.status == 200 and
                      {'red', 'blue', 'referee'} <= names and
                      'referee_online' in roles,
                      f'roles={sorted(names)} referee_online={roles.get("referee_online")}')

            async with session.ws_connect(base + '/ws/referee') as referee:
                first = await recv_first(referee, {'ready', 'occupied'})
                if first.get('type') == 'occupied':
                    check('ws /ws/referee ready', False,
                          '裁判角色已被占用：请先关闭浏览器/编辑器里的裁判页面再运行本脚本')
                    print('\nSUMMARY: 裁判角色被占用，后续用例无法执行')
                    return 1
                status = await recv_until(referee, 'status')
                check('referee status stream', 'match' in status and 'robots' in status,
                      f"state={status['match']['state']} robots={len(status['robots'])}")

                # The camera only frames while running; drive the match to READY.
                state = int(status['match']['state'])
                ok, detail = await drive_to_ready(referee, state)
                check('referee reset -> READY', ok, detail)

                # --- player websocket: keyboard/mouse input -------------------
                async with session.ws_connect(base + '/ws/red') as socket:
                    ready = await recv_first(socket, {'ready', 'occupied'})
                    if ready.get('type') == 'occupied':
                        check('ws /ws/red ready', False,
                              '红方角色已被占用：请先关闭对应的选手页面再运行本脚本')
                        return 1
                    token = ready['token']
                    check('ws /ws/red ready', ready.get('role') == 'red' and
                          ready.get('player_robot') == 'red_infantry_robot',
                          f"robot={ready.get('player_robot')}")

                    async with session.ws_connect(base + '/ws/red') as second:
                        occupied = await recv_until(second, 'occupied')
                        check('role exclusivity', occupied.get('role') == 'red')

                    await socket.send_json({'type': 'ping', 'sent_at': 12345})
                    pong = await recv_until(socket, 'pong')
                    check('ws ping/pong', pong.get('sent_at') == 12345)

                    await socket.send_json({'type': 'input', 'sequence': 7, 'active': True,
                                            'mouse_dx': 12.5, 'mouse_dy': -3.0,
                                            'pressed_keys': ['KeyW', 'MouseLeft']})
                    got = probe.wait_for(
                        lambda: any(m.active and m.sequence == 7 for m in probe.messages))
                    sample = probe.messages[-1] if probe.messages else None
                    check('keyboard/mouse -> player_input', got and sample is not None and
                          'KeyW' in sample.pressed_keys and 'MouseLeft' in sample.pressed_keys and
                          abs(sample.mouse_dx - 12.5) < 1e-3,
                          f'pressed={list(sample.pressed_keys) if sample else None} '
                          f'dx={sample.mouse_dx if sample else None}')

                    # --- WebRTC video offer ------------------------------------
                    peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
                    peer.addTransceiver('video', direction='recvonly')
                    await peer.setLocalDescription(await peer.createOffer())
                    async with session.post(base + '/api/webrtc/offer', json={
                            'role': 'red', 'token': token,
                            'sdp': peer.localDescription.sdp,
                            'type': peer.localDescription.type}) as response:
                        answer = await response.json()
                    video_line = 'm=video' in answer.get('sdp', '')
                    check('POST /api/webrtc/offer', response.status == 200 and video_line,
                          f"status={response.status} type={answer.get('type')} "
                          f'm=video:{video_line}')
                    await peer.setRemoteDescription(
                        RTCSessionDescription(sdp=answer['sdp'], type=answer['type']))

                    frame_ok, detail = False, ''
                    try:
                        frame = await drain_while(socket, asyncio.create_task(
                            read_video_frame(peer)))
                        frame_ok = frame is not None and frame.width > 0
                        detail = f'{frame.width}x{frame.height}'
                    except Exception as exc:  # noqa: BLE001 - report whatever failed
                        detail = f'{type(exc).__name__}: {exc}'
                    check('WebRTC decoded video frame', frame_ok, detail)
                    await peer.close()

                    try:
                        await socket.send_json({'type': 'release'})
                        await recv_until(socket, 'released')
                    except (ConnectionError, aiohttp.ClientError) as exc:
                        check('player release acknowledged', False, str(exc))

                # --- referee start (READY -> RUNNING) -------------------------
                await referee.send_json({'type': 'referee_command', 'request_id': 2,
                                         'command': 'start'})
                result = await recv_until(referee, 'referee_result', timeout=15)
                check('referee start accepted', result.get('accepted') is True,
                      str(result.get('message')))
    finally:
        probe.close()

    failed = [name for name, ok, _ in RESULTS if not ok]
    print(f'\nSUMMARY: {len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed')
    if failed:
        print('FAILED: ' + ', '.join(failed))
    return 1 if failed else 0


async def _wait_track(peer):
    for _ in range(300):
        for receiver in peer.getReceivers():
            if receiver.track is not None:
                return receiver.track
        await asyncio.sleep(0.05)
    raise TimeoutError('no remote track')


if __name__ == '__main__':
    raise SystemExit(asyncio.run(main()))
