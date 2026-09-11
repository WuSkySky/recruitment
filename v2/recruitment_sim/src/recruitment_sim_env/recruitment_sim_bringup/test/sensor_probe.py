#!/usr/bin/env python3
"""Sample and summarise camera, IMU and GPU-lidar output from robot domains."""
import math
import struct
import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2


def make_node(domain, name):
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node(name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return node, context, executor


def main():
    image_node, image_context, image_executor = make_node(20, 'sensor_probe_image')
    imu_node, imu_context, imu_executor = make_node(20, 'sensor_probe_imu')
    lidar_node, lidar_context, lidar_executor = make_node(21, 'sensor_probe_lidar')
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)

    images, imus, clouds = [], [], []
    image_node.create_subscription(Image, '/red/infantry/camera/image', images.append, qos)
    imu_node.create_subscription(Imu, '/red/infantry/gimbal_imu', imus.append, qos)
    lidar_node.create_subscription(PointCloud2, '/red/sentry/livox/lidar', clouds.append, qos)

    def spin_all(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            image_executor.spin_once(timeout_sec=0.005)
            imu_executor.spin_once(timeout_sec=0.005)
            lidar_executor.spin_once(timeout_sec=0.005)

    spin_all(20)
    assert images and imus and clouds, f'image={len(images)} imu={len(imus)} lidar={len(clouds)}'

    # --- camera: save a frame and report colour spread -----------------------
    frame = images[-1]
    from PIL import Image as PILImage
    PILImage.frombytes('RGB', (frame.width, frame.height), bytes(frame.data)).save(sys.argv[1])
    channel_spread = tuple(
        max(bytes(frame.data)[i::3]) - min(bytes(frame.data)[i::3]) for i in range(3))
    print(f'camera: {frame.width}x{frame.height} {frame.encoding} frame_id={frame.header.frame_id} '
          f'stamp={frame.header.stamp.sec}.{frame.header.stamp.nanosec:09d} rgb_spread={channel_spread}')

    # --- IMU ----------------------------------------------------------------
    imu = imus[-1]
    accel = math.sqrt(imu.linear_acceleration.x ** 2 + imu.linear_acceleration.y ** 2 +
                      imu.linear_acceleration.z ** 2)
    norm = math.sqrt(imu.orientation.x ** 2 + imu.orientation.y ** 2 +
                     imu.orientation.z ** 2 + imu.orientation.w ** 2)
    print(f'imu: frame_id={imu.header.frame_id} |accel|={accel:.2f} quat_norm={norm:.3f}')

    # --- lidar: structure, range and a top-down occupancy view --------------
    cloud = clouds[-1]
    points = list(struct.iter_unpack('<fff', cloud.data))
    finite = [p for p in points if all(math.isfinite(v) for v in p)]
    ranges = [math.sqrt(p[0] ** 2 + p[1] ** 2) for p in finite]
    print(f'lidar: {cloud.width}x{cloud.height} point_step={cloud.point_step} '
          f'row_step={cloud.row_step} frame_id={cloud.header.frame_id} '
          f'finite={len(finite)}/{len(points)} range=[{min(ranges):.2f}, {max(ranges):.2f}]')

    size = 32
    span = 16.0
    grid = [[' '] * size for _ in range(size)]
    for x, y, z in finite:
        if not -0.6 <= z <= 2.5:
            continue
        col = int((x + span / 2) / span * (size - 1))
        row = int((span / 2 - y) / span * (size - 1))
        if 0 <= row < size and 0 <= col < size:
            grid[row][col] = '#' if grid[row][col] == ' ' else '@'
    print(f'lidar top-down occupancy ({span:.0f}x{span:.0f} m, z in [-0.6, 2.5]):')
    for row in grid:
        print('  |' + ''.join(row) + '|')

    for node, context in ((image_node, image_context), (imu_node, imu_context),
                          (lidar_node, lidar_context)):
        node.destroy_node()
        context.shutdown()
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
