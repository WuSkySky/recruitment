# Recruitment Simulation

面向招新的 ROS 2 / Gazebo Fortress 双机器人仿真环境，提供 `pb2025_infantry_robot` 和
`pb2025_sentry_robot` 的模型、传感器、底盘、云台、射击与灯条控制。当前版本不包含裁判
系统，但保留装甲碰撞结构与弹丸物理检测，便于后续接入重构后的裁判模块。

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

工作空间只包含 4 个 ROS 包：

- `recruitment_sim_interfaces`：灯条服务。
- `recruitment_sim_description`：机器人与 RMUL 2026 场地描述、模型资源、SDF→URDF 工具和 Gazebo 插件。
- `recruitment_sim_robot_base`：底盘、云台、射击、灯条与里程计控制节点。
- `recruitment_sim_bringup`：机器人配置、生成、传感器桥接和 RViz 启动。

## 启动

默认在 RMUL 2026 3V3 场地中同时启动红色步兵和蓝色哨兵：

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
`name`、受支持的 `type`、队伍/灯条 `color` 以及 `x/y/z/yaw` 初始位姿。ROS 话题前缀由
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
```

## 裁判系统边界

当前代码不启动或依赖裁判系统，也不发布 `/referee_system/*`。发射插件仍负责生成弹丸、
检测任意物理碰撞以及碰撞/超时后的弹丸回收；“是否击中有效装甲、扣血和比赛状态”留给
后续裁判系统重构实现。

第三方代码与模型来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
