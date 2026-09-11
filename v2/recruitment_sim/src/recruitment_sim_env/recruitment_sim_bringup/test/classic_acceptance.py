#!/usr/bin/env python3
"""Integration acceptance against a running default Classic bringup.

Run with install-classic sourced. This drives and resets the simulation;
use a dedicated validation instance, never an active competition.
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import Bool, Float64
from recruitment_sim_interfaces.msg import MatchInfo, RobotStatus, SimulationFrame
from recruitment_sim_interfaces.srv import ControlMatch, SetRobotEnabled, SetLightColor


class Acceptance:
    def __init__(self, single=False):
        rclpy.init()
        self.internal = Node('classic_acceptance')
        self.nodes = [self.internal]
        self.contexts = []
        self.latest, self.count = {}, {}
        self.subs = []
        self.watch(self.internal, MatchInfo, '/referee_system/match/info', 'match', transient=True)
        self.watch(self.internal, SimulationFrame, '/referee_system/simulation/frame', 'frame')
        robots = [('red/infantry', 20)] if single else [
            ('red/infantry', 20), ('red/sentry', 21), ('blue/infantry', 30), ('blue/sentry', 31)]
        for ns, domain in robots:
            ctx = Context()
            rclpy.init(context=ctx, domain_id=domain)
            node = Node(f'classic_acceptance_{domain}', context=ctx)
            self.nodes.append(node)
            self.contexts.append(ctx)
            self.watch(node, Image, f'/{ns}/camera/image', f'{ns}/image')
            self.watch(node, Imu, f'/{ns}/gimbal_imu', f'{ns}/imu')
            self.watch(node, Odometry, f'/{ns}/chassis_odometry', f'{ns}/odom')
            if 'sentry' in ns:
                self.watch(node, PointCloud2, f'/{ns}/livox/lidar', f'{ns}/lidar')
        self.robot = self.nodes[1]
        self.velocity = self.robot.create_publisher(Twist, '/red/infantry/cmd_chassis_vel', 10)
        self.yaw = self.robot.create_publisher(Float64, '/red/infantry/cmd_yaw_vel', 10)
        self.shoot = self.robot.create_publisher(Bool, '/red/infantry/cmd_shoot', 10)
        self.watch(self.robot, Float64, '/red/infantry/feedback_yaw_angle', 'yaw')
        self.watch(self.internal, RobotStatus, '/referee_system/red_infantry_robot/status', 'status', transient=True)
        self.control = self.internal.create_client(ControlMatch, '/referee_system/match/control')
        self.enable = self.internal.create_client(SetRobotEnabled, '/referee_system/red_infantry_robot/set_enabled')
        self.light = self.internal.create_client(SetLightColor, '/red/infantry/robot_base/set_light_color')
        self.expected_sensors = [key for key in self.count if '/' in key]

    def watch(self, node, typ, topic, key, transient=False):
        self.count[key] = 0
        def receive(msg):
            self.latest[key] = msg
            self.count[key] += 1
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        if transient:
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs.append(node.create_subscription(typ, topic, receive, qos))

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for node in self.nodes:
                rclpy.spin_once(node, timeout_sec=0.001)

    def until(self, predicate, description, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.spin(.02)
        raise AssertionError(f'timed out: {description}; match={self.latest.get("match")}')

    def call(self, client, req):
        self.until(client.service_is_ready, 'service discovery')
        future = client.call_async(req)
        self.until(future.done, 'service completion')
        response = future.result()
        assert response is not None
        assert getattr(response, 'accepted', getattr(response, 'success', False)), response
        return response

    def command(self, value):
        return self.call(self.control, ControlMatch.Request(command=value))

    def run(self, resets):
        self.until(lambda: all(self.count[k] for k in self.expected_sensors) and 'match' in self.latest,
                   'all domain sensors', 120)
        for key in self.expected_sensors:
            msg = self.latest[key]
            if key.endswith('/image'):
                expected = (1920, 1080, 'rgb8') if 'sentry' in key else (640, 480, 'rgb8')
                assert (msg.width, msg.height, msg.encoding) == expected, key
                assert len(msg.data) == msg.step * msg.height
                assert max(msg.data) > min(msg.data), 'blank camera'
            elif key.endswith('/lidar'):
                assert msg.height == 32 and msg.width > 0
                assert len(msg.data) == msg.row_step * msg.height
        for node in self.nodes[1:]:
            services = dict(node.get_service_names_and_types())
            assert '/referee_system/match/control' not in services
            assert '/referee_system/simulation/control' not in services
        self.spin(2)
        original = self.latest['red/infantry/odom'].pose.pose.position
        start = (original.x, original.y)
        cmd = Twist(); cmd.linear.x = .3
        for _ in range(10):
            self.velocity.publish(cmd); self.spin(.1)
        self.velocity.publish(Twist()); self.spin(.5)
        current = self.latest['red/infantry/odom'].pose.pose.position
        assert math.hypot(current.x-start[0], current.y-start[1]) > .03, 'chassis did not move'
        # 空闲时云台必须保持不动：Classic 缺少 fmax 时云台会因重力缓慢扭动。
        self.yaw.publish(Float64(data=0.)); self.spin(.2)
        idle_before = self.latest['yaw'].data
        self.spin(2.5)
        idle_drift = abs(self.latest['yaw'].data - idle_before)
        assert idle_drift < .05, f'gimbal drifted while idle ({idle_drift:.3f} rad)'
        # 速度指令必须真正驱动关节，而不只是产生微弱漂移。
        before = self.latest['yaw'].data
        start = time.monotonic()
        for _ in range(10):
            self.yaw.publish(Float64(data=.4)); self.spin(.1)
        elapsed = max(time.monotonic() - start, .1)
        self.yaw.publish(Float64(data=0.)); self.spin(.3)
        rate = (self.latest['yaw'].data - before) / elapsed
        assert rate > .2, f'gimbal did not follow its velocity command (rate={rate:.3f} rad/s)'
        before = self.latest['status'].total_shots
        self.shoot.publish(Bool(data=True)); self.spin(1)
        self.shoot.publish(Bool(data=False)); self.spin(.5)
        assert self.latest['status'].total_shots > before, 'no actual shots recorded'
        self.call(self.enable, SetRobotEnabled.Request(target=3, enabled=False))
        before = self.latest['status'].total_shots
        self.shoot.publish(Bool(data=True)); self.spin(.5)
        assert self.latest['status'].total_shots == before, 'disabled shooter fired'
        for index in range(resets):
            if self.latest['match'].state not in (MatchInfo.TRAINING, MatchInfo.FINISHED, MatchInfo.ERROR):
                self.command(1)
                self.until(lambda: self.latest['match'].state == MatchInfo.FINISHED, 'FINISHED')
            previous_count = dict(self.count)
            self.command(2)
            self.until(lambda: self.latest['match'].state == MatchInfo.READY, 'READY')
            self.until(lambda: all(self.count[k] > previous_count[k] for k in self.expected_sensors),
                       'sensors after reset')
            assert self.latest['status'].total_shots == 0
            self.command(0)
            self.until(lambda: self.latest['match'].state == MatchInfo.RUNNING, 'RUNNING')
            self.command(1)
            self.until(lambda: self.latest['match'].state == MatchInfo.FINISHED, 'FINISHED')
            self.call(self.light, SetLightColor.Request(color=3))
            print(f'reset {index+1}/{resets}: READY -> RUNNING -> FINISHED; paused light OK', flush=True)
        return {'resets': resets, 'message_counts': self.count}

    def close(self):
        self.velocity.publish(Twist()); self.yaw.publish(Float64(data=0.)); self.shoot.publish(Bool(data=False))
        self.spin(.1)
        for node in self.nodes:
            node.destroy_node()
        for ctx in self.contexts:
            ctx.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--resets', type=int, default=20)
    parser.add_argument('--single', action='store_true')
    args = parser.parse_args()
    acceptance = Acceptance(args.single)
    try:
        print(json.dumps(acceptance.run(args.resets), indent=2))
    finally:
        acceptance.close()
