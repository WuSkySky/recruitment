#!/usr/bin/env python3

import argparse

import rclpy
from recruitment_sim_interfaces.srv import SetLightColor


def main():
    parser = argparse.ArgumentParser(description="Set a simulated robot's light-bar color")
    parser.add_argument(
        "color",
        choices=("none", "red", "blue", "yellow", "white"),
        help="target light-bar color",
    )
    args, ros_args = parser.parse_known_args()

    colors = {
        "none": SetLightColor.Request.NONE,
        "red": SetLightColor.Request.RED,
        "blue": SetLightColor.Request.BLUE,
        "yellow": SetLightColor.Request.YELLOW,
        "white": SetLightColor.Request.WHITE,
    }

    rclpy.init(args=ros_args)
    node = rclpy.create_node("test_light_color")
    client = node.create_client(SetLightColor, "set_light_color")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("set_light_color service is unavailable")
        rclpy.shutdown()
        raise SystemExit(1)

    request = SetLightColor.Request()
    request.color = colors[args.color]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if response is None or not response.success:
        message = "service call failed" if response is None else response.message
        node.get_logger().error(message)
        rclpy.shutdown()
        raise SystemExit(1)

    node.get_logger().info(response.message)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
