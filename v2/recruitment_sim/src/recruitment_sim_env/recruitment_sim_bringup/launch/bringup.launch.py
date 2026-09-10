#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from xmacro.xmacro4sdf import XMLMacro4sdf


ROBOT_TOPIC_TYPES = {
    "pb2025_infantry_robot": "infantry",
    "pb2025_sentry_robot": "sentry",
}
SUPPORTED_ROBOTS = set(ROBOT_TOPIC_TYPES)
SUPPORTED_COLORS = {"none", "red", "blue", "yellow", "white"}
ROBOT_DOMAIN_IDS = {
    "red/infantry": "20", "red/sentry": "21",
    "blue/infantry": "30", "blue/sentry": "31",
}


def _robot_namespace(robot):
    return f"{robot['color']}/{ROBOT_TOPIC_TYPES[robot['type']]}"


def _bridge_mapping(robot_name, robot_namespace, robot_type, world_name):
    model_prefix = f"/world/{world_name}/model/{robot_name}"
    ros_prefix = f"/{robot_namespace}"
    mappings = [
        (f"/{robot_name}/odometry", f"{ros_prefix}/chassis_odometry", "nav_msgs/msg/Odometry", "ignition.msgs.Odometry"),
        (f"{model_prefix}/link/gimbal_pitch/sensor/gimbal_imu/imu", f"{ros_prefix}/gimbal_imu", "sensor_msgs/msg/Imu", "ignition.msgs.IMU"),
        (f"{model_prefix}/link/front_industrial_camera/sensor/front_industrial_camera/image", f"{ros_prefix}/camera/image", "sensor_msgs/msg/Image", "ignition.msgs.Image"),
        (f"{model_prefix}/link/front_industrial_camera/sensor/front_industrial_camera/camera_info", f"{ros_prefix}/camera/camera_info", "sensor_msgs/msg/CameraInfo", "ignition.msgs.CameraInfo"),
    ]
    if robot_type == "pb2025_sentry_robot":
        mappings.extend(
            [
                (f"{model_prefix}/link/front_mid360/sensor/front_mid360_lidar/scan/points", f"{ros_prefix}/livox/lidar", "sensor_msgs/msg/PointCloud2", "ignition.msgs.PointCloudPacked"),
            ]
        )
    return mappings


def _validate_config(config):
    if not isinstance(config, dict) or not isinstance(config.get("robots"), list):
        raise RuntimeError("robots config must contain a 'robots' list")
    if not config["robots"]:
        raise RuntimeError("robots config must contain at least one robot")
    zone = config.get("control_zone", {})
    if not isinstance(zone, dict):
        raise RuntimeError("control_zone must be a mapping")
    bounds = zone.get("bounds", [-1.5, 1.5, -1.5, 1.5])
    if (
        not isinstance(bounds, list)
        or len(bounds) != 4
        or any(not isinstance(v, (int, float)) or isinstance(v, bool) for v in bounds)
        or bounds[0] >= bounds[1]
        or bounds[2] >= bounds[3]
    ):
        raise RuntimeError("control_zone.bounds must be [min_x, max_x, min_y, max_y]")

    names = set()
    namespaces = set()
    for index, robot in enumerate(config["robots"]):
        if not isinstance(robot, dict):
            raise RuntimeError(f"robots[{index}] must be a mapping")
        missing = {"name", "type", "color", "pose", "referee"} - robot.keys()
        if missing:
            raise RuntimeError(f"robots[{index}] is missing: {', '.join(sorted(missing))}")
        if robot["name"] in names:
            raise RuntimeError(f"duplicate robot name: {robot['name']}")
        names.add(robot["name"])
        if robot["type"] not in SUPPORTED_ROBOTS:
            raise RuntimeError(f"unsupported robot type: {robot['type']}")
        if robot["color"] not in SUPPORTED_COLORS:
            raise RuntimeError(f"unsupported robot color: {robot['color']}")
        robot_namespace = _robot_namespace(robot)
        if robot_namespace in namespaces:
            raise RuntimeError(f"duplicate robot topic namespace: /{robot_namespace}")
        namespaces.add(robot_namespace)
        if not isinstance(robot["pose"], dict):
            raise RuntimeError(f"robots[{index}].pose must be a mapping")
        missing_pose = {"x", "y", "z", "yaw"} - robot["pose"].keys()
        if missing_pose:
            raise RuntimeError(
                f"robots[{index}].pose is missing: {', '.join(sorted(missing_pose))}"
            )
        if not isinstance(robot["referee"], dict):
            raise RuntimeError(f"robots[{index}].referee must be a mapping")
        missing_referee = {"max_hp", "heat_limit", "cooling_rate"} - robot["referee"].keys()
        if missing_referee:
            raise RuntimeError(
                f"robots[{index}].referee is missing: "
                f"{', '.join(sorted(missing_referee))}"
            )
        referee = robot["referee"]
        if not isinstance(referee["max_hp"], int) or referee["max_hp"] <= 0:
            raise RuntimeError(f"robots[{index}].referee.max_hp must be a positive integer")
        for field in ("heat_limit", "cooling_rate"):
            value = referee[field]
            if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
                raise RuntimeError(
                    f"robots[{index}].referee.{field} must be a positive number"
                )


def _spawn_robots(context: LaunchContext):
    bringup_share = get_package_share_directory("recruitment_sim_bringup")
    description_share = get_package_share_directory("recruitment_sim_description")
    robots_file = LaunchConfiguration("robots_file").perform(context)
    log_level = LaunchConfiguration("log_level")

    with open(robots_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    _validate_config(config)
    world_name = config.get("world_name", "default")
    base_params = os.path.join(bringup_share, "config", "base_params.yaml")

    actions = []
    model_sdfs = []
    for robot in config["robots"]:
        robot_name = robot["name"]
        robot_type = robot["type"]
        robot_namespace = _robot_namespace(robot)
        pose = robot["pose"]
        xmacro_path = os.path.join(
            description_share, "resource", "xmacro", f"{robot_type}.sdf.xmacro"
        )

        xmacro = XMLMacro4sdf()
        xmacro.set_xml_file(xmacro_path)
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_sdf = xmacro.to_string()
        root = ET.fromstring(robot_sdf)
        model = root.find("model")
        model.set("name", robot_name)
        model_pose = model.find("pose")
        if model_pose is None:
            model_pose = ET.SubElement(model, "pose")
        model_pose.text = f"{pose['x']} {pose['y']} {pose['z']} 0 0 {pose['yaw']}"
        model_sdfs.append(ET.tostring(root, encoding="unicode"))

        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{robot_name}",
                output="screen",
                arguments=[
                    "-string", robot_sdf,
                    "-name", robot_name,
                    "-allow_renaming", "false",
                    "-x", str(pose["x"]),
                    "-y", str(pose["y"]),
                    "-z", str(pose["z"]),
                    "-Y", str(pose["yaw"]),
                ],
            )
        )
        actions.append(
            Node(
                package="recruitment_sim_robot_base",
                executable="robot_base",
                namespace=robot_namespace,
                output="screen",
                parameters=[
                    base_params,
                    {"robot_name": robot_name, "world_name": world_name},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            )
        )
        mappings = _bridge_mapping(robot_name, robot_namespace, robot_type, world_name)
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{robot_name}_bridge",
                additional_env=(
                    {"ROS_DOMAIN_ID": ROBOT_DOMAIN_IDS[robot_namespace]}
                    if robot_namespace in ROBOT_DOMAIN_IDS else {}
                ),
                output="screen",
                arguments=[
                    f"{gz_topic}@{ros_type}[{gz_type}"
                    for gz_topic, _, ros_type, gz_type in mappings
                ],
                remappings=[
                    (gz_topic, ros_topic)
                    for gz_topic, ros_topic, _, _ in mappings
                ],
            )
        )

    actions.append(
        Node(
            package="recruitment_sim_referee_system",
            executable="referee_system",
            name="referee_system",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_sdfs": ParameterValue(model_sdfs, value_type=List[str]),
                    "zone_enabled": os.path.basename(LaunchConfiguration("world_file").perform(context)) != "empty_world.sdf",
                    "zone_bounds": [float(v) for v in config.get("control_zone", {}).get("bounds", [-1.5, 1.5, -1.5, 1.5])],
                    "robot_names": [robot["name"] for robot in config["robots"]],
                    "robot_teams": [robot["color"] for robot in config["robots"]],
                    "robot_max_hps": [robot["referee"]["max_hp"] for robot in config["robots"]],
                    "robot_heat_limits": [
                        float(robot["referee"]["heat_limit"]) for robot in config["robots"]
                    ],
                    "robot_cooling_rates": [
                        float(robot["referee"]["cooling_rate"])
                        for robot in config["robots"]
                    ],
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )
    )

    for namespace, domain in ROBOT_DOMAIN_IDS.items():
        team = namespace.split("/")[0]
        robots = [robot for robot in config["robots"] if _robot_namespace(robot) == namespace]
        if robots:
            actions.append(Node(
                package="recruitment_sim_bringup",
                executable="team_topic_bridge",
                name=f"{namespace.replace('/', '_')}_topic_bridge",
                output="screen",
                parameters=[{
                    "team": team,
                    "team_domain": int(domain),
                    "robot_namespaces": [_robot_namespace(robot) for robot in robots],
                    "robot_names": [robot["name"] for robot in robots],
                }],
            ))

    return actions


def generate_launch_description():
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) in {int(d) for d in ROBOT_DOMAIN_IDS.values()}:
        raise RuntimeError("Internal ROS_DOMAIN_ID must differ from robot domains 20, 21, 30 and 31")
    bringup_share = get_package_share_directory("recruitment_sim_bringup")
    description_share = get_package_share_directory("recruitment_sim_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    default_world = os.path.join(
        description_share, "resource", "worlds", "rmul_2026h_world.sdf"
    )
    default_robots = os.path.join(bringup_share, "config", "robots.yaml")
    default_rviz = os.path.join(bringup_share, "rviz", "visualize_robot.rviz")

    world_file = LaunchConfiguration("world_file")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("rviz")
    use_player_web = LaunchConfiguration("player_web")

    gz_launch_path = os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        condition=IfCondition(gui),
        launch_arguments={"gz_args": [world_file, " -r"]}.items(),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        condition=UnlessCondition(gui),
        launch_arguments={"gz_args": [world_file, " -r -s"]}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robots_file", default_value=default_robots),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("player_web", default_value="false"),
            DeclareLaunchArgument("player_web_port", default_value="8080"),
            DeclareLaunchArgument("log_level", default_value="info"),
            gazebo_gui,
            gazebo_headless,
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="clock_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            ),
            OpaqueFunction(function=_spawn_robots),
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(use_rviz),
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            ),
            Node(
                package="recruitment_sim_player_web",
                executable="player_web",
                name="player_web",
                condition=IfCondition(use_player_web),
                output="screen",
                parameters=[
                    {
                        "robots_file": LaunchConfiguration("robots_file"),
                        "port": ParameterValue(
                            LaunchConfiguration("player_web_port"), value_type=int
                        ),
                    }
                ],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            ),
        ]
    )
