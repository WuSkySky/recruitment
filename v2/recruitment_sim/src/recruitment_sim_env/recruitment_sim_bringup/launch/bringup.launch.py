#!/usr/bin/env python3

import os
import glob
import tempfile
import shutil
import math
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepend_path(name, entry):
    entries = [item for item in os.environ.get(name, "").split(os.pathsep) if item]
    if entry not in entries:
        entries.insert(0, entry)
    return os.pathsep.join(entries)


def _configure_gazebo_environment():
    """Make Gazebo Classic resolve this project's resources from any shell.

    Classic finds ``model://`` assets through the environment. A stale env hook
    (for example a build tree left over from the Fortress backend) leaves these
    variables unset, so Classic falls back to the online model database and
    startup stalls on models.gazebosim.org. ``xmacro4sdf`` also reads
    ``GAZEBO_MODEL_PATH`` while it is being imported, so the variables must be
    in place before the import below runs.

    The system Gazebo media path is added as well; without it Classic cannot
    find ``media/rtshaderlib`` and renders with shader generation disabled.
    """
    description_share = get_package_share_directory("recruitment_sim_description")
    models = os.path.join(description_share, "resource", "models")
    prefix = os.path.dirname(os.path.dirname(description_share))
    entries = {
        "GAZEBO_MODEL_PATH": [models],
        "SDF_PATH": [models],
        "GAZEBO_RESOURCE_PATH": [description_share],
        "GAZEBO_PLUGIN_PATH": [os.path.join(prefix, "plugins")],
    }
    for media in sorted(glob.glob("/usr/share/gazebo-[0-9]*")):
        entries["GAZEBO_RESOURCE_PATH"].append(media)
        system_models = os.path.join(media, "models")
        if os.path.isdir(system_models):
            entries["GAZEBO_MODEL_PATH"].append(system_models)
    entries["GAZEBO_PLUGIN_PATH"].extend(
        sorted(glob.glob("/usr/lib/*/gazebo-[0-9]*/plugins")))
    for name, paths in entries.items():
        for path in reversed(paths):
            os.environ[name] = _prepend_path(name, path)
    # The project ships every model it needs, so never contact the online model
    # database: a network stall there is indistinguishable from a hang.
    os.environ["GAZEBO_MODEL_DATABASE_URI"] = ""


_configure_gazebo_environment()

from xmacro.xmacro4sdf import XMLMacro4sdf  # noqa: E402 - must follow the env setup


ROBOT_TOPIC_TYPES = {
    "pb2025_infantry_robot": "infantry",
    "pb2025_sentry_robot": "sentry",
}
SUPPORTED_ROBOTS = set(ROBOT_TOPIC_TYPES)
SUPPORTED_COLORS = {"none", "red", "blue", "yellow", "white"}
LIGHT_BAR_RGBA = {
    "none": "0 0 0 1",
    "red": "1 0 0 1",
    "blue": "0 0 1 1",
    "yellow": "1 1 0 1",
    "white": "1 1 1 1",
}
ROBOT_DOMAIN_IDS = {
    "red/infantry": "20", "red/sentry": "21",
    "blue/infantry": "30", "blue/sentry": "31",
}


def _robot_namespace(robot):
    return f"{robot['color']}/{ROBOT_TOPIC_TYPES[robot['type']]}"


def apply_light_bar_color(root, color):
    """Seed the configured light-bar colour into the spawned model SDF.

    The module definitions ship their light bars as plain white; the plugin
    recolours them at runtime, but Gazebo Classic rebuilds GUI visuals from the
    model SDF (for example after a reset), which would drop back to white. Baking
    the colour into the SDF keeps the bars correct from the first frame.
    """
    rgba = LIGHT_BAR_RGBA[color]
    for visual in root.iter("visual"):
        if visual.get("name") != "light_bar_visual":
            continue
        material = visual.find("material")
        if material is None:
            material = ET.SubElement(visual, "material")
        for tag in ("ambient", "diffuse", "emissive"):
            element = material.find(tag)
            if element is None:
                element = ET.SubElement(material, tag)
            element.text = rgba


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
        noise = robot.get("projectile_noise", {})
        if not isinstance(noise, dict):
            raise RuntimeError(f"robots[{index}].projectile_noise must be a mapping")
        for field in ("yaw_angle_variance", "pitch_angle_variance"):
            value = noise.get(field, 0.0)
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value < 0):
                raise RuntimeError(
                    f"robots[{index}].projectile_noise.{field} must be finite and non-negative"
                )
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
    infrastructure_log_level = LaunchConfiguration("infrastructure_log_level")

    with open(robots_file, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    _validate_config(config)
    world_name = config.get("world_name", "default")
    base_params = os.path.join(bringup_share, "config", "base_params.yaml")

    actions = []
    temp_dir = tempfile.mkdtemp(prefix="recruitment_classic_")
    def cleanup(context, *args, **kwargs):
        shutil.rmtree(temp_dir, ignore_errors=True)
        return []
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    with open(base_params, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["/**"]["ros__parameters"]
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
        noise = robot.get("projectile_noise", {})
        xmacro.generate({
            "global_initial_color": robot["color"],
            "projectile_yaw_angle_variance": noise.get("yaw_angle_variance", 0.0),
            "projectile_pitch_angle_variance": noise.get("pitch_angle_variance", 0.0),
        })
        robot_sdf = xmacro.to_string()
        root = ET.fromstring(robot_sdf)
        apply_light_bar_color(root, robot["color"])
        model = root.find("model")
        model.set("name", robot_name)
        model_pose = model.find("pose")
        if model_pose is None:
            model_pose = ET.SubElement(model, "pose")
        model_pose.text = "0 0 0 0 0 0"
        plugin = model.find("plugin")
        def add(parent, name, value):
            ET.SubElement(parent, name).text = str(value).lower() if isinstance(value, bool) else str(value)
        add(plugin, "namespace", "/" + robot_namespace)
        domain = int(ROBOT_DOMAIN_IDS.get(robot_namespace, os.environ.get("ROS_DOMAIN_ID", "0")))
        add(plugin, "sensor_domain", domain)
        add(plugin, "use_odometry", params.get("use_odometry", False))
        add(plugin, "initial_color", ["none", "red", "blue", "yellow", "white"].index(robot["color"]))
        noise_element = ET.SubElement(plugin, "noise")
        for group in ("actuator_noise", "sensor_noise"):
            for key, value in params[group].items():
                if not math.isfinite(value) or value < 0:
                    raise RuntimeError(f"{group}.{key} must be finite and nonnegative")
                add(noise_element, group.removesuffix("_noise") + "_" + key, value)
        for link in model.findall("link"):
            for sensor in link.findall("sensor"):
                sensor_type = sensor.get("type")
                if sensor_type not in {"camera", "imu", "gpu_ray"}:
                    continue
                sensor_plugin = ET.SubElement(sensor, "plugin", {
                    "name": "recruitment_sensor", "filename": "libRecruitmentSimSensors.so"})
                add(sensor_plugin, "namespace", "/" + robot_namespace)
                add(sensor_plugin, "domain", domain)
                if sensor_type == "camera":
                    frame_id = sensor.findtext("camera/optical_frame_id", link.get("name"))
                    camera_image = sensor.find("camera/image")
                    if camera_image.find("format") is None:
                        add(camera_image, "format", "R8G8B8")
                elif sensor_type == "gpu_ray":
                    frame_id = link.get("name")
                else:
                    frame_id = f"{robot_name}/{link.get('name')}/{sensor.get('name')}"
                add(sensor_plugin, "frame_id", frame_id)
        robot_path = os.path.join(temp_dir, robot_name + ".sdf")
        ET.ElementTree(root).write(robot_path, encoding="unicode")
        actions.append(Node(
            package="gazebo_ros", executable="spawn_entity.py", name=f"spawn_{robot_name}",
            output="screen", arguments=[
                "-file", robot_path, "-entity", robot_name,
                "-x", str(pose["x"]), "-y", str(pose["y"]), "-z", str(pose["z"]),
                "-Y", str(pose["yaw"]), "-timeout", "120",
                "--ros-args", "--log-level", infrastructure_log_level,
            ],
        ))

    actions.append(
        Node(
            package="recruitment_sim_referee_system",
            executable="referee_system",
            name="referee_system",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
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
                arguments=[
                    "--ros-args", "--log-level", infrastructure_log_level,
                ],
            ))

    if LaunchConfiguration("player_web").perform(context).lower() in ("true", "1"):
        infantry_colors = {
            robot["color"] for robot in config["robots"]
            if ROBOT_TOPIC_TYPES[robot["type"]] == "infantry"
        }
        if {"red", "blue"} <= infantry_colors:
            actions.append(Node(
                package="recruitment_sim_player_web",
                executable="player_web",
                name="player_web",
                output="screen",
                parameters=[
                    {
                        "robots_file": LaunchConfiguration("robots_file"),
                        "port": ParameterValue(
                            LaunchConfiguration("player_web_port"), value_type=int
                        ),
                        "red_camera_domain_id": int(ROBOT_DOMAIN_IDS["red/infantry"]),
                        "blue_camera_domain_id": int(ROBOT_DOMAIN_IDS["blue/infantry"]),
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ))
        else:
            actions.append(LogInfo(msg=(
                "player_web skipped: the contest terminal needs one red and one blue "
                "infantry robot in robots_file; partial rosters run without Web."
            )))

    return actions


def generate_launch_description():
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) in {int(d) for d in ROBOT_DOMAIN_IDS.values()}:
        raise RuntimeError("Internal ROS_DOMAIN_ID must differ from robot domains 20, 21, 30 and 31")
    bringup_share = get_package_share_directory("recruitment_sim_bringup")
    description_share = get_package_share_directory("recruitment_sim_description")
    gazebo_share = get_package_share_directory("gazebo_ros")
    default_world = os.path.join(
        description_share, "resource", "worlds", "rmul_2026h_world.sdf"
    )
    default_robots = os.path.join(bringup_share, "config", "robots.yaml")
    default_rviz = os.path.join(bringup_share, "rviz", "visualize_robot.rviz")

    world_file = LaunchConfiguration("world_file")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("rviz")

    # Gazebo resource paths were pinned in os.environ before the xmacro import,
    # so the gzserver/gzclient processes below inherit them.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"world": world_file, "gui": gui, "verbose": "true"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robots_file", default_value=default_robots),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("player_web", default_value="true"),
            DeclareLaunchArgument("player_web_port", default_value="8080"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "infrastructure_log_level", default_value="warn"
            ),
            gazebo,
            OpaqueFunction(function=_spawn_robots),
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(use_rviz),
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            ),
        ]
    )
