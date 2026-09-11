# Recruitment Simulation

面向招新的 ROS 2 / Gazebo Classic 11 多机器人仿真环境，提供 `pb2025_infantry_robot` 和
`pb2025_sentry_robot` 的模型、传感器、底盘、云台、射击与灯条控制，并提供血量、射击热量、
自动装甲命中、胜利点、中央占点、比赛状态机和执行器失能等裁判功能。

## 环境与构建

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11.10.2（Ubuntu 包），gazebo_ros_pkgs 3.9.0

```bash
cd /home/skysky/workspaces/recruitment/v2/recruitment_sim
# 如曾构建过 Fortress 版本，先删除旧目录，否则会加载不兼容的 Ignition 插件
rm -rf build install
python3 -m pip install -r requirements.txt
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --build-base build-classic --install-base install-classic \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install-classic/setup.bash
```

Gazebo Classic 已结束上游维护。本项目固定使用 Ubuntu 22.04 / ROS 2 Humble / Gazebo 11，
并使用独立的 `build-classic` / `install-classic` 目录，避免与 Fortress 时期的
`build` / `install` 混用。**如果 source 了旧的 install 目录，`GAZEBO_MODEL_PATH` 会缺失，
Gazebo 会转去下载在线模型库并卡在 `models.gazebosim.org`**，因此升级时务必先删除旧目录。

启动文件会在导入 XMacro 之前自行设置 `GAZEBO_MODEL_PATH`、`SDF_PATH`、
`GAZEBO_RESOURCE_PATH`、`GAZEBO_PLUGIN_PATH`，并把 `GAZEBO_MODEL_DATABASE_URI` 置空，
因此即使当前 shell 没有 source 过 `/usr/share/gazebo-11/setup.sh` 也能正常解析本地
`model://` 资源、找到 OGRE shader 库，并且不会访问网络。

工作空间包含 6 个 ROS 包：

- `recruitment_sim_interfaces`：机器人/比赛状态消息、比赛控制、裁判使能和灯条服务。
- `recruitment_sim_description`：机器人与 RMUL 2026 场地描述、模型资源、SDF→URDF 工具和 Gazebo 插件。
- `recruitment_sim_robot_base`：基础控制算法、噪声、里程计和使能状态库，以及测试工具；由机器人插件调用。
- `recruitment_sim_referee_system`：血量、热量、命中、中央占点、胜利点和比赛生命周期。
- `recruitment_sim_player_web`：红蓝选手端、Web 裁判端、WebRTC 相机和原始键鼠输入网关。
- `recruitment_sim_bringup`：机器人配置、生成、分域小消息网关和 RViz 启动。

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

默认同时启动统一浏览器比赛终端，端口为 `8080`。如需指定其他端口：

```bash
ros2 launch recruitment_sim_bringup bringup.launch.py \
  player_web_port:=8081
```

浏览器访问 `http://<仿真主机局域网地址>:8080`，在选择页分别打开红方选手端、蓝方选手端
和裁判端。三个角色可同时连接；同一角色只允许一个标签页占用。选手点击进入比赛后页面进入
全屏并锁定鼠标，标签页失焦、退出鼠标锁定或断线时只释放对应阵营的输入。Web 服务监听成功后，
终端日志会显示本机和可检测到的局域网访问网址。

默认隐藏机器人生成器和小消息网关的常规 INFO 日志，核心节点仍保持 INFO 级别。如需查看完整
基础设施日志，可传入 `infrastructure_log_level:=info`。

如不需要 Web 端，可显式关闭：

```bash
ros2 launch recruitment_sim_bringup bringup.launch.py player_web:=false
```

比赛终端需要红蓝双方各一台步兵。如果 `robots_file` 里缺少任一方的步兵（例如只启动单台
机器人），启动会打印 `player_web skipped: ...` 并跳过 Web 节点，仿真其余部分照常运行；
此时也可以继续显式传入 `player_web:=false` 来消除提示。

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
默认世界的中央控制区边界为 `[-1.5, 1.5, -1.5, 1.5]`（依次为最小/最大 x、最小/最大 y），
可通过 YAML 顶层 `control_zone.bounds` 覆盖。空场世界自动关闭占点检测。

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
- `/<team>/<type>/cmd_shoot`：`std_msgs/msg/Bool` 射击开关；`true` 时以固定 `18 m/s` 弹速持续射击，
  最小间隔 50 ms，必须显式发送 `false` 才会停止。**实际射速上限约 5 发/秒**：Gazebo Classic
  每 200 ms（墙钟）才处理一次实体插入消息（`World::processMsgsPeriod`），所以插件必须等上一枚
  弹丸真正生成后再发下一枚，否则多枚弹丸会在同一仿真时刻、同一炮口位置被创建而互相碰撞销毁。
- `/<team>/<type>/robot_base/set_light_color`：灯条颜色服务，0–4 对应关闭、红、蓝、黄、白。
  机器人插件通过 Classic `Visual` 消息更新实际渲染材质，暂停仿真时也接受改色。
  修改插件并重新构建后需要重启 Gazebo 才会加载新的动态库；仅重新调用服务不会更新已加载的插件。
- `/referee_system/<robot_name>/set_enabled`：裁判控制服务；目标 0–3 分别为整机、底盘、云台、
  发射机构。失能会立即停止对应执行器，传感器和反馈继续运行。
- `/<team>/<type>/chassis_odometry`、`gimbal_imu`：状态反馈。`chassis_odometry` 以 100 Hz 发布，
  云台反馈（`feedback_*`）同样以 100 Hz 发布，`gimbal_imu` 为传感器自身速率（约 190 Hz）。
- `/<team>/<type>/camera/image`、`camera_info`：工业相机。
- `/<team>/infantry/player_input`：浏览器以 60 Hz 发布的 `PlayerInput` 原始键鼠状态；
  `mouse_dx/mouse_dy` 是当前周期内浏览器 `movementX/movementY` 的总和，正方向分别为右和下。
  `active=false` 表示页面未锁定鼠标或控制会话已经断开，选手控制节点应立即输出安全的零命令。
  `pressed_keys` 是当前按住的键鼠名称数组（`string[]`），去重并按名称排序；无按键或输入失效时为空。
  键盘使用浏览器 `KeyboardEvent.code`（例如 `KeyW`、`ShiftLeft`），鼠标按钮编号 0–4 分别为
  `MouseLeft`、`MouseMiddle`、`MouseRight`、`MouseBack`、`MouseForward`。
  Esc 释放鼠标、F3 切换诊断面板，均不进入数组；系统保留快捷键及 Fn 等不保证可捕获。
  只发送按住状态，不缓存按键事件，两次采样之间完成的极短点击可能漏掉。
  Python 订阅回调可用 `msg.active and "KeyW" in msg.pressed_keys` 判断操作。
  原 `key_w/key_a/key_s/key_d/left_button/right_button` 字段已移除，外部订阅节点须同步修改、
  重新构建接口和依赖包，并与网页及后端一起更新、重启。

- 哨兵额外提供 `/<team>/sentry/livox/lidar`。
- 当前不发布 `/tf`、`/tf_static` 或 `joint_states`。

执行器命令和反馈支持零均值正态噪声，可在
`recruitment_sim_bringup/config/base_params.yaml` 中配置方差。`actuator_noise` 分别控制底盘速度、
云台 pitch/yaw 速度命令实际送入 Gazebo 前的噪声；`sensor_noise` 分别控制底盘速度反馈、
云台 pitch/yaw 速度反馈和角度反馈的噪声。底盘的 `linear.x`、`linear.y`、`angular.z` 分别由
`chassis_x_velocity_variance`、`chassis_y_velocity_variance`、`chassis_yaw_velocity_variance`
配置，并独立采样。方差必须是有限非负数，默认均为 `0.0`（关闭噪声）。底盘速度反馈噪声作用于
内部 `robot_base/odom.twist.twist`，因此仅在 `use_odometry: true` 时可见。
内部 `robot_base/odom` 的底盘位姿反馈通过 `chassis_x_position_increment_variance`、
`chassis_y_position_increment_variance` 和 `chassis_yaw_increment_variance` 配置每次发布时车体
前向、横向和转角增量的噪声方差。带噪增量从上一反馈位姿继续积分，因此误差会持续累计，且
yaw 漂移会改变后续位移的积分方向。
首次启动以及比赛重置后，内部噪声里程计 `robot_base/odom` 的平面 `x/y/yaw` 均从零开始。每台机器人提供内部生命周期
服务 `/referee_system/<robot_name>/initialize_odometry`；裁判系统在 Gazebo 回位后调用该服务，
并等待全部机器人确认后才恢复比赛。初始化完成前不会继续发布上一轮的里程计位姿。

弹丸初速度方向噪声在 `recruitment_sim_bringup/config/robots.yaml` 中按机器人配置：

```yaml
projectile_noise:
  yaw_angle_variance: 0.0
  pitch_angle_variance: 0.0
```

两个参数单位均为 rad²，必须是有限非负数；省略或设为 `0.0` 时不产生方向误差。
每次实际发弹时，在枪口局部坐标系独立采样零均值正态 yaw/pitch 偏角，保持初速度大小
18 m/s 不变，之后仍由 Gazebo 处理重力和碰撞，不在飞行中继续加噪声。
例如方差 `0.0001` 对应标准差 `0.01 rad`（约 `0.57°`）。修改配置后需重启仿真。
这种误差不累计，比赛重置不重置或重新播种随机数生成器。

### 机器人分域与小消息网关


每台机器人的 Classic 传感器插件使用独立 ROS Context 指定 ROS 域：红步兵为 `20`、红哨兵为 `21`、蓝步兵为 `30`、蓝哨兵为 `31`。
相机、camera_info、云台 IMU、哨兵点云及真值里程计 `chassis_odometry` 均由插件直接发布到
对应机器人域；其他颜色沿用启动环境中的域。话题名称保持不变，不增加 ROS→ROS 转发。

在选手电脑或主机上，可分别查看四个域（`--no-daemon` 避免复用其他域的 CLI daemon）：

```bash
ROS_DOMAIN_ID=20 ros2 topic list --no-daemon
ROS_DOMAIN_ID=21 ros2 topic list --no-daemon
ROS_DOMAIN_ID=30 ros2 topic list --no-daemon
ROS_DOMAIN_ID=31 ros2 topic list --no-daemon
```

主机为每台红蓝机器人启动一个小消息网关。内部域继承启动环境的 `ROS_DOMAIN_ID`（默认 `0`），
必须不同于 `20`、`21`、`30`、`31`。网关只按下面的白名单、方向转发：

| 方向 | 接口 |
| --- | --- |
| 机器人域 → 内部域 | 对应机器人 `cmd_chassis_vel`、`cmd_yaw_vel`、`cmd_pitch_vel`、`cmd_shoot` |
| 内部域 → 机器人域 | 对应机器人四个 `feedback_*` 角度/速度反馈 |
| 内部域 → 机器人域 | 对应机器人 `/referee_system/<robot_name>/status`、对应步兵 `player_input` |
| 内部域 → 四个机器人域 | `/clock`、`/referee_system/match/status`（仅比赛阶段和已进行秒数，保留 transient-local；`match/info` 不转发） |

机器人控制插件与中央裁判继续在内部域通信，重置、使能、灯条以及其他服务均不转发。
相机、IMU、点云和真值里程计由 Classic 插件直接发布到各机器人域，不经过小消息网关。
反馈保留在内部域，并单向转发到对应机器人域。选手可在域 20/21/30/31 直接发送己方控制命令。

Web 主节点仍在内部域处理裁判状态、控制服务和键鼠输入；另为红、蓝选手端分别创建位于步兵
机器人域的相机订阅节点，因此相机图像不经过 ROS→ROS 网关即可进入 WebRTC 画面。
本方案只用于逻辑分组，不提供身份认证或访问控制；修改域编号仍可接入其他域。

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
热量上限会锁定发射机构，热量回到 0 后解锁，达到“上限 + 100”后在本局内
永久锁定。敌方 `target_collision` 装甲命中每次扣 20 HP，同一装甲 50 ms 内只结算一次；
友军和自身命中完全忽略，血量归零后自动整机失能。

启动后处于 `TRAINING`，血量、热量和受击正常工作，但不计比赛时间和胜利点。比赛服务与状态为：

- `/referee_system/match/control`：`ControlMatch` 服务；命令 `0` 开始裁判、`1` 人工结束、`2` 重置并恢复仿真。
- `/referee_system/match/info`：10 Hz、可靠且 transient-local 的 `MatchInfo`，保留原完整消息内容，包含时间戳、状态、
  已进行/剩余时间、双方胜利点、攻击伤害、总剩余血量、占领方、有效占领机器人、结果和错误；仅在内部域发布，供网页后端等内部组件订阅，不转发到选手域。
- `/referee_system/match/status`：10 Hz、可靠且 transient-local 的 `MatchStatus`，仅包含 `uint8 state`（比赛阶段）和
  `float64 elapsed_seconds`（比赛已进行的仿真秒数），转发到四个选手域 20、21、30、31。阶段枚举值保持不变。

```bash
ros2 service call /referee_system/match/control \
  recruitment_sim_interfaces/srv/ControlMatch "{command: 2}"
# 等待 MatchStatus.state 进入 READY（6），再开始裁判计时
ros2 service call /referee_system/match/control \
  recruitment_sim_interfaces/srv/ControlMatch "{command: 0}"
ros2 topic echo /referee_system/match/info
```

比赛时长 300 秒，双方从 200 胜利点开始。存活机器人的底盘中心进入中央 `3 × 3 m` 区域后按
先到先得占领；离区保留 2 秒，占领每满 1 秒扣对方 1 点。机器人首次战亡使己方扣 20 点。
胜利点归零立即结束；时间耗尽后依次比较胜利点、全队实际攻击伤害和总剩余血量，仍相同则平局。
人工结束的结果为 `ABORTED`。结束会失能机器人并暂停 Gazebo。`RESUME` 会清除旧弹丸，恢复出生
位姿、满血、零热量、200:200、执行器初始状态及 `robots.yaml` 配置的装甲板灯条颜色，再恢复
仿真并进入 `READY`；`START` 只从该状态
开始 300 秒裁判计时，不再隐式重置。生命周期复位保留传感器并原地重置机器人，
通过 Classic API 恢复关节、速度和控制状态，等待旧弹丸实际删除后才继续初始化。

本阶段不包含撞击伤害、42 mm 弹丸、回血复活、弹量限制、射击初速度处罚、准备阶段和 BO 管理。

## Web 比赛终端开发

已构建的静态资源随 ROS 包安装，运行比赛不需要 Node.js。修改 Vue 前端后重新生成资源：

```bash
cd src/recruitment_sim_env/recruitment_sim_player_web/frontend
npm ci
npm test
npm run build
```

红蓝选手 HUD 显示双方剩余得分点、比赛时间、双方步兵/哨兵血量、本机血量和固定中央准星，
按 `F3` 可临时查看相机接收/显示帧率、WebRTC 丢帧、WebSocket RTT 和连接状态。裁判端显示
比赛阶段、时间、比分、占点、四台机器人状态，并按比赛状态提供重置、开始和结束按钮。

第三方代码与模型来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## Classic 插件结构

每台机器人由一个 `RecruitmentSimRobot` ModelPlugin 处理控制和内部服务，直接调用基础算法库。
`RecruitmentSimSensors` 为相机、IMU 和 GPU 雷达提供 ROS 输出；共享运行库按域管理 Context
和执行器。没有独立 robot_base 进程或 ROS–Gazebo 传感器桥。

`RecruitmentSimRefereeSimulation` WorldPlugin 在物理步结束时汇总有序发弹/命中事件。
`/referee_system/simulation/frame` 使用 `SimulationFrame`，控制服务使用 `ControlSimulation`；
它们只存在于内部域。生命周期请求按 token 去重，轮次隔离保留，暂停时仍可处理重置和改色。
原 V1 字符串通信已移除。插件内部的 Classic 材质消息不经过 ROS 网关。

`chassis_odometry` 始终是机器人域的平面真值输出；`use_odometry` 仅控制内部域
`robot_base/odom` 的累计噪声输出。两者保持迁移前代码的实际行为。
Gazebo 11 本身依赖 ignition-math 等底层库，这不意味着项目仍运行 Ignition Gazebo。

## 迁移基线与验收

- [docs/classic-baseline.md](docs/classic-baseline.md)：迁移时的 ROS 域划分、选手/内部接口、
  网关白名单、传感器 QoS、关键配置与构建方法。
- [docs/classic-verification.md](docs/classic-verification.md)：干净全量构建、单元测试、
  四机器人/单机器人重置验收、分域隔离、GUI/RViz、Web 与传感器抽样的完整记录。

`recruitment_sim_bringup/test/` 下有三个需要正在运行的仿真才能执行的手动验收脚本：

```bash
source install-classic/setup.bash
ROS_DOMAIN_ID=101 python3 src/recruitment_sim_env/recruitment_sim_bringup/test/classic_acceptance.py --resets 20
ROS_DOMAIN_ID=101 python3 src/recruitment_sim_env/recruitment_sim_bringup/test/web_acceptance.py --port 8080
ROS_DOMAIN_ID=101 python3 src/recruitment_sim_env/recruitment_sim_bringup/test/sensor_probe.py camera.png
```

它们不注册进 `colcon test`，因为需要一个正在运行的仿真；自动化单元测试仍由 `colcon test`
覆盖。请在专用验证实例上运行这些脚本，不要对正在进行的比赛执行。

