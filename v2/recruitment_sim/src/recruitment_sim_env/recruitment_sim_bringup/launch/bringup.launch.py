#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from recruitment_sim_description.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


SUPPORTED_ROBOTS = {"pb2025_infantry_robot", "pb2025_sentry_robot"}
SUPPORTED_COLORS = {"none", "red", "blue", "yellow", "white"}


def _bridge_mapping(robot_name, robot_type, world_name):
    model_prefix = f"/world/{world_name}/model/{robot_name}"
    mappings = [
        (f"/{robot_name}/odometry", f"/{robot_name}/chassis_odometry_gt", "nav_msgs/msg/Odometry", "ignition.msgs.Odometry"),
        (f"{model_prefix}/joint_state", f"/{robot_name}/joint_states", "sensor_msgs/msg/JointState", "ignition.msgs.Model"),
        (f"{model_prefix}/link/chassis/sensor/chassis_imu/imu", f"/{robot_name}/chassis_imu", "sensor_msgs/msg/Imu", "ignition.msgs.IMU"),
        (f"{model_prefix}/link/gimbal_pitch/sensor/gimbal_imu/imu", f"/{robot_name}/gimbal_imu", "sensor_msgs/msg/Imu", "ignition.msgs.IMU"),
        (f"{model_prefix}/link/front_industrial_camera/sensor/front_industrial_camera/image", f"/{robot_name}/front_industrial_camera/image", "sensor_msgs/msg/Image", "ignition.msgs.Image"),
        (f"{model_prefix}/link/front_industrial_camera/sensor/front_industrial_camera/camera_info", f"/{robot_name}/front_industrial_camera/camera_info", "sensor_msgs/msg/CameraInfo", "ignition.msgs.CameraInfo"),
    ]
    if robot_type == "pb2025_sentry_robot":
        mappings.extend(
            [
                (f"{model_prefix}/link/front_mid360/sensor/front_mid360_lidar/scan/points", f"/{robot_name}/livox/lidar", "sensor_msgs/msg/PointCloud2", "ignition.msgs.PointCloudPacked"),
                (f"{model_prefix}/link/front_mid360/sensor/front_mid360_imu/imu", f"/{robot_name}/livox/imu", "sensor_msgs/msg/Imu", "ignition.msgs.IMU"),
            ]
        )
    return mappings


def _validate_config(config):
    if not isinstance(config, dict) or not isinstance(config.get("robots"), list):
        raise RuntimeError("robots config must contain a 'robots' list")
    if not config["robots"]:
        raise RuntimeError("robots config must contain at least one robot")

    names = set()
    for index, robot in enumerate(config["robots"]):
        if not isinstance(robot, dict):
            raise RuntimeError(f"robots[{index}] must be a mapping")
        missing = {"name", "type", "color", "pose"} - robot.keys()
        if missing:
            raise RuntimeError(f"robots[{index}] is missing: {', '.join(sorted(missing))}")
        if robot["name"] in names:
            raise RuntimeError(f"duplicate robot name: {robot['name']}")
        names.add(robot["name"])
        if robot["type"] not in SUPPORTED_ROBOTS:
            raise RuntimeError(f"unsupported robot type: {robot['type']}")
        if robot["color"] not in SUPPORTED_COLORS:
            raise RuntimeError(f"unsupported robot color: {robot['color']}")
        if not isinstance(robot["pose"], dict):
            raise RuntimeError(f"robots[{index}].pose must be a mapping")
        missing_pose = {"x", "y", "z", "yaw"} - robot["pose"].keys()
        if missing_pose:
            raise RuntimeError(
                f"robots[{index}].pose is missing: {', '.join(sorted(missing_pose))}"
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
    for robot in config["robots"]:
        robot_name = robot["name"]
        robot_type = robot["type"]
        pose = robot["pose"]
        xmacro_path = os.path.join(
            description_share, "resource", "xmacro", f"{robot_type}.sdf.xmacro"
        )

        xmacro = XMLMacro4sdf()
        xmacro.set_xml_file(xmacro_path)
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_sdf = xmacro.to_string()

        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_sdf)
        robot_urdf = urdf_generator.to_string()

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
                namespace=robot_name,
                output="screen",
                parameters=[
                    base_params,
                    {"robot_name": robot_name, "world_name": world_name},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            )
        )
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=robot_name,
                output="screen",
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                parameters=[
                    {"use_sim_time": True, "robot_description": robot_urdf}
                ],
            )
        )

        mappings = _bridge_mapping(robot_name, robot_type, world_name)
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{robot_name}_bridge",
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

    return actions


def generate_launch_description():
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
        ]
    )
