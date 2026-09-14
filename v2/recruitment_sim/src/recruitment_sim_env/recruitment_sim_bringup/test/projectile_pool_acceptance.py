#!/usr/bin/env python3
"""Acceptance test for a running projectile-pool test bringup."""
import argparse
import json
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from recruitment_sim_interfaces.msg import MatchInfo, SimulationFrame
from recruitment_sim_interfaces.srv import ControlMatch


class Acceptance:
    def __init__(self, robot_name, namespace, shoot_domain):
        rclpy.init()
        self.robot_name = robot_name
        self.node = Node('projectile_pool_acceptance')
        self.frame, self.match, self.events = None, None, []
        frame_qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(SimulationFrame, '/referee_system/simulation/frame',
                                      self.on_frame, frame_qos)
        self.node.create_subscription(MatchInfo, '/referee_system/match/info',
                                      lambda msg: setattr(self, 'match', msg), state_qos)
        self.shoot_context = Context()
        rclpy.init(context=self.shoot_context, domain_id=shoot_domain)
        self.shoot_node = Node('projectile_pool_shoot', context=self.shoot_context)
        self.shoot = self.shoot_node.create_publisher(Bool, f'/{namespace}/cmd_shoot', 10)
        self.control = self.node.create_client(ControlMatch, '/referee_system/match/control')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.worker = threading.Thread(target=self.executor.spin, daemon=True)
        self.worker.start()

    def on_frame(self, message):
        self.frame = message
        self.events.extend((message.round_id, message.stamp_ns, event)
                           for event in message.events if event.shooter == self.robot_name)

    def wait(self, predicate, description, timeout=60):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError(f'timed out: {description}')

    def sim_publish(self, value, duration_ns):
        start = self.frame.stamp_ns
        message = Bool(data=value)
        while self.frame.stamp_ns - start < duration_ns:
            self.shoot.publish(message)
            time.sleep(.005)

    def command(self, value):
        self.wait(self.control.service_is_ready, 'control service')
        future = self.control.call_async(ControlMatch.Request(command=value))
        self.wait(future.done, 'control response')
        assert future.result() and future.result().accepted, future.result()

    def reset(self):
        # END is idempotent and also avoids acting on an older transient-local
        # MatchInfo sample received during discovery.
        previous_info = self.match
        self.command(ControlMatch.Request.END)
        self.wait(lambda: self.match is not previous_info, 'fresh match state after END')
        self.wait(lambda: self.match.state == MatchInfo.FINISHED, 'FINISHED before reset')
        previous_round = self.frame.round_id
        previous_info = self.match
        self.command(ControlMatch.Request.RESUME)
        self.wait(lambda: self.match is not previous_info, 'fresh match state after RESUME')
        self.wait(lambda: self.match.state == MatchInfo.READY, 'READY after reset')
        self.wait(lambda: self.frame.round_id > previous_round, 'new round')

    def run(self, duration):
        self.wait(lambda: self.frame is not None and self.frame.ready, 'projectile pool ready')
        self.wait(lambda: self.match is not None, 'match state')
        self.wait(lambda: self.shoot.get_subscription_count() > 0, 'shoot subscriber')
        self.reset()
        # Warm up DDS discovery before choosing the measurement window. Otherwise
        # the first few commands can legitimately be dropped by a late subscriber.
        self.sim_publish(False, 200_000_000)
        initial_round, begin = self.frame.round_id, len(self.events)
        self.sim_publish(True, int(duration * 1e9))
        self.sim_publish(False, 300_000_000)
        events = [item for item in self.events[begin:] if item[0] == initial_round]
        shots = [(stamp, event) for _, stamp, event in events if event.kind == event.SHOT]
        hits = [event for _, _, event in events if event.kind == event.HIT]
        minimum = 115
        assert len(shots) >= minimum, f'only {len(shots)} shots in {duration}s'
        ids = [event.projectile_id for _, event in shots]
        assert ids == list(range(ids[0], ids[0] + len(ids))), 'non-contiguous shot IDs'
        deltas = [(right[0] - left[0]) / 1e9 for left, right in zip(shots, shots[1:])]
        assert all(.045 <= delta <= .055 for delta in deltas), (
            f'shot interval {min(deltas):.6f}..{max(deltas):.6f}s')
        assert hits, 'no projectile collision events'
        assert len(hits) == len({event.projectile_id for event in hits}), 'duplicate HIT'

        self.reset()
        reset_round, reset_begin = self.frame.round_id, len(self.events)
        self.sim_publish(False, 200_000_000)
        assert not [item for item in self.events[reset_begin:] if item[0] == reset_round], (
            'old event leaked into reset round')
        self.sim_publish(True, 20_000_000)
        self.sim_publish(False, 50_000_000)
        self.wait(lambda: any(r == reset_round and e.kind == e.SHOT
                              for r, _, e in self.events[reset_begin:]), 'shot after reset')
        reset_shots = [e for r, _, e in self.events[reset_begin:]
                       if r == reset_round and e.kind == e.SHOT]
        assert reset_shots[0].projectile_id == 0, 'shot ID did not reset'
        return {'shots': len(shots), 'hits': len(hits),
                'interval_min': min(deltas), 'interval_max': max(deltas),
                'reset_round': reset_round}

    def close(self):
        self.shoot.publish(Bool(data=False))
        self.executor.shutdown()
        self.worker.join(timeout=5)
        self.node.destroy_node()
        self.shoot_node.destroy_node()
        self.shoot_context.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='pool_test_sentry_robot')
    parser.add_argument('--namespace', default='red/sentry')
    parser.add_argument('--shoot-domain', type=int, default=21)
    parser.add_argument('--duration', type=float, default=6.2)
    args = parser.parse_args()
    test = Acceptance(args.robot_name, args.namespace, args.shoot_domain)
    try:
        print(json.dumps(test.run(args.duration), indent=2), flush=True)
    finally:
        test.close()
