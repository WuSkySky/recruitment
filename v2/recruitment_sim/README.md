# Recruitment Simulation

面向招新的 ROS 2 / Gazebo Fortress 多机器人仿真环境，提供 `pb2025_infantry_robot` 和
`pb2025_sentry_robot` 的模型、传感器、底盘、云台、射击与灯条控制，并提供血量、射击热量、
自动装甲命中和执行器失能等基础裁判功能。

## 环境与构建

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress / Ignition Gazebo 6

```bash
cd /home/skysky/workspaces/recruitment/v2/recruitment_sim
python3 -m pip install -r requirements.txt
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

工作空间包含 5 个 ROS 包：

- `recruitment_sim_interfaces`：机器人状态消息、裁判使能和灯条服务。
- `recruitment_sim_description`：机器人与 RMUL 2026 场地描述、模型资源、SDF→URDF 工具和 Gazebo 插件。
- `recruitment_sim_robot_base`：底盘、云台、射击、灯条与里程计控制节点。
- `recruitment_sim_referee_system`：血量、热量、命中判定和失能控制。
- `recruitment_sim_bringup`：机器人配置、生成、传感器桥接和 RViz 启动。

## 启动

默认在 RMUL 2026 3V3 场地中启动红蓝双方各一台步兵和一台哨兵，共四台机器人。每方两台
机器人均在己方启动区短边居中，并沿长边前后排列；步兵在靠场地中心的前位，哨兵在后位：
两台机器人车头也沿启动区长边朝向场地中心。

```bash
ros2 launch recruitment_sim_bringup bringup.launch.py
```

无界面运行：

```bash
ros2 launch recruitment_sim_bringup bringup.launch.py gui:=false
```

同时启动 RViz：

```bash
ros2 launch recruitment_sim_bringup bringup.launch.py rviz:=true
```

如需切换回空场，可显式指定：

```bash
ros2 launch recruitment_sim_bringup bringup.launch.py \
  world_file:=$(ros2 pkg prefix recruitment_sim_description)/share/recruitment_sim_description/resource/worlds/empty_world.sdf
```

机器人列表位于
`src/recruitment_sim_env/recruitment_sim_bringup/config/robots.yaml`。每项必须提供唯一的
`name`、受支持的 `type`、队伍/灯条 `color`、裁判参数 `referee` 以及 `x/y/z/yaw` 初始位姿。
`referee` 必须给出正数 `max_hp`、`heat_limit` 和 `cooling_rate`。ROS 话题前缀由
`color/type` 决定，例如红方步兵为 `/red/infantry`、蓝方哨兵为 `/blue/sentry`；同一配置
中不允许出现重复前缀。也可以通过 `robots_file:=/absolute/path/robots.yaml` 加载其他配置。

## 控制接口

将下方 `<team>/<type>` 替换为队伍颜色和机器人类型，例如 `red/infantry` 或
`blue/sentry`：

- `/<team>/<type>/cmd_chassis_vel`：`geometry_msgs/msg/Twist` 底盘速度；`linear.x`、`linear.y`
  和 `angular.z` 均按底盘坐标系解释，底盘不会自动跟随云台。
- `/<team>/<type>/cmd_yaw_vel`、`cmd_pitch_vel`：`std_msgs/msg/Float64` 云台 yaw/pitch
  速度命令，单位为 `rad/s`。命令会一直生效，必须显式发送 `0.0` 才会停止。
- `/<team>/<type>/feedback_yaw_vel`、`feedback_pitch_vel`：`std_msgs/msg/Float64`
  云台实际关节速度，单位为 `rad/s`，以 100 Hz 发布。
- `/<team>/<type>/feedback_yaw_angle`、`feedback_pitch_angle`：`std_msgs/msg/Float64`
  云台相对初始朝前位置的单圈角度，单位为 `rad`，范围为 `[-π, π]`，以 100 Hz 发布。
- `/<team>/<type>/cmd_shoot`：`std_msgs/msg/Bool` 射击开关；`true` 时以 50 ms
  间隔和固定 `18 m/s` 弹速持续射击，必须显式发送 `false` 才会停止。
- `/<team>/<type>/robot_base/set_light_color`：灯条颜色服务，0–4 对应关闭、红、蓝、黄、白。
- `/referee_system/<robot_name>/set_enabled`：裁判控制服务；目标 0–3 分别为整机、底盘、云台、
  发射机构。失能会立即停止对应执行器，传感器和反馈继续运行。
- `/<team>/<type>/chassis_odometry_gt`、`gimbal_imu`：状态反馈。
- `/<team>/<type>/front_industrial_camera/image`、`camera_info`：工业相机。
- 哨兵额外提供 `/<team>/sentry/livox/lidar`。
- 当前不发布 `/tf`、`/tf_static` 或 `joint_states`。

交互式测试工具示例：

```bash
ros2 topic pub --once /red/infantry/cmd_chassis_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.0}}"
ros2 topic pub --once /red/infantry/cmd_yaw_vel std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic echo /red/infantry/feedback_yaw_angle
ros2 topic pub --once /red/infantry/cmd_yaw_vel std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic pub --once /red/infantry/cmd_shoot std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /red/infantry/cmd_shoot std_msgs/msg/Bool "{data: false}"
ros2 run recruitment_sim_robot_base test_light_color.py yellow --ros-args -r __ns:=/red/infantry/robot_base
ros2 service call /referee_system/red_infantry_robot/set_enabled \
  recruitment_sim_interfaces/srv/SetRobotEnabled "{target: 0, enabled: false}"
```

## 裁判系统

默认 bringup 会启动中央裁判节点。每台机器人以 10 Hz 在
`/referee_system/<robot_name>/status` 发布 `recruitment_sim_interfaces/msg/RobotStatus`，其中包含
血量、热量、最近 100 ms 实际发弹数、累计发弹/命中数和过热状态。例如：

```bash
ros2 topic echo /referee_system/red_infantry_robot/status
```

当前仅实现 17 mm 弹丸规则：实际生成一发弹丸增加 10 热量，每 100 ms 按配置冷却；超过
热量上限会锁定发射机构，热量回到 0 后解锁，达到“上限 + 100”后在本次裁判节点生命周期内
永久锁定。敌方 `target_collision` 装甲命中每次扣 20 HP，同一装甲 50 ms 内只结算一次；
友军和自身命中完全忽略，血量归零后自动整机失能。

本阶段不包含撞击伤害、42 mm 弹丸、回血复活、比赛阶段、弹量限制和射击初速度处罚。

第三方代码与模型来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
