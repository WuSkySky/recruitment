# 使用说明

## 1. 机器人已有功能说明

默认阵容是红蓝双方各一台步兵、一台哨兵，共四台机器人。所有机器人共用一个模型框架。话题中的 `<team>/<type>` 取值为 `red/infantry`、`red/sentry`、
`blue/infantry`、`blue/sentry`。

### 1.1 底盘

- **麦轮全向底盘**，可以同时做前后、左右平移和自转。
- 指令话题 `/<team>/<type>/cmd_chassis_vel`（`geometry_msgs/msg/Twist`）。`linear.x`、`linear.y`
  和 `angular.z` 都按**底盘坐标系**解释，单位分别为 m/s、m/s、rad/s。
- 里程计 `/<team>/<type>/robot_base/odom`

### 1.2 云台

- 由 `gimbal_yaw_joint` 和 `gimbal_pitch_joint` 两个关节组成：**yaw 可以连续旋转**，
  **pitch 有机械限位，约 −45° ~ +32.4°**（`-3.14*0.25` ~ `3.14*0.18`）。
- 速度指令 `/<team>/<type>/cmd_yaw_vel`、`cmd_pitch_vel`（`std_msgs/msg/Float64`，单位 rad/s）。
  和底盘一样是持续生效的，必须显式发送 `0.0` 才会停止。
- 反馈话题（均为 `std_msgs/msg/Float64`

### 1.3 发射

- 射击开关 `/<team>/<type>/cmd_shoot`（`std_msgs/msg/Bool`）：`true` 时持续射击，必须显式发送
  `false` 才会停止。
- **固定弹速 18 m/s**，最小发射间隔 50 ms（标称 20 发/秒）。
- 射击热量与过热锁枪由裁判系统负责
- 战亡后热量清零；复活后处于「虚弱」时发射机构锁定，进入己方补给区才解除

### 1.4 传感器

| 传感器 | 装备 | 话题 | 类型 | 配置频率 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 工业相机 | 步兵、哨兵 | `camera/image`、`camera/camera_info` | `sensor_msgs/msg/Image`、`CameraInfo` | 30 Hz | 编码 `rgb8`；步兵 640×480，哨兵 1920×1080 |
| 云台 IMU | 步兵、哨兵 | `gimbal_imu` | `sensor_msgs/msg/Imu` | 200 Hz | 姿态四元数、角速度、线加速度 |
| Mid360 雷达 | 哨兵 | `livox/lidar` | `sensor_msgs/msg/PointCloud2` | 20 Hz | 1875 × 32 点，量程 0.1–40 m，含 `intensity` |


### 1.5 裁判系统

所有裁判通讯都在 `/referee_system` 前缀下

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/referee_system/<robot_name>/status` | `msg/RobotStatus` | 血量、热量、实发弹数、累计发弹/命中、存活、过热与锁枪，以及复活读条、无敌与虚弱状态，10 Hz |
| `/referee_system/match/status` | `msg/MatchStatus` | 比赛阶段 `state` 和已进行秒数 `elapsed_seconds`，10 Hz |

`msg/RobotStatus` 里与复活相关的字段：`alive`、`invincible`、`weakened`、`death_count`、
`revive_remaining_seconds`、`invincible_remaining_seconds`。战亡后 `revive_remaining_seconds`
倒数到 0 即原地复活；`weakened=true` 表示发射机构锁定且无法占领控制区，开进**己方补给区**
（红 `x∈[-6.0,-4.5] y∈[2.0,4.0]`，蓝对称）即可解除，同时开始按每秒 25% 上限血量回血。

### 1.6 选手端与裁判端

浏览器访问 `http://<仿真主机地址>:8080` 进入选择页，可选红方选手端、蓝方选手端和裁判端，

**选手端**（第一视角操作界面）：
- 给操作手显示本方步兵的第一视角画面与比赛 HUD，并把键鼠操作发布到对应的域。

**裁判端**（比赛控制台）：

- 显示比赛阶段、剩余时间、红蓝双方剩余得分点、四台机器人生命值、中央控制区占领方和比赛结果。
- 提供「重置比赛」「开始比赛」「结束比赛」三个按钮，按当前比赛阶段自动禁用不可用的操作。
---

## 2. 目录结构

| 功能包 | 职责 |
| --- | --- |
| `recruitment_sim_interfaces` | 机器人状态、比赛状态、比赛控制、裁判使能与灯条服务接口 |
| `recruitment_sim_description` | 机器人与场地描述、模型资源、SDF→URDF 工具、Gazebo Classic 插件 |
| `recruitment_sim_robot_base` | 底盘、云台、射击、灯条与里程计的基础算法库；由插件调用，不注册独立节点 |
| `recruitment_sim_referee_system` | 血量、热量、命中、中央占点、胜利点与比赛生命周期 |
| `recruitment_sim_player_web` | 红蓝选手端、Web 裁判端、WebRTC 相机与键鼠输入网关 |
| `recruitment_sim_bringup` | 机器人配置与生成、分域小消息网关、RViz 启动 |


---

## 3. 话题介绍

`<team>/<type>` 按机器人替换，例如 `/red/infantry/cmd_chassis_vel`。

### 3.1 控制指令（选手 → 机器人）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/<team>/<type>/cmd_chassis_vel` | `geometry_msgs/msg/Twist` | 底盘速度，底盘坐标系 |
| `/<team>/<type>/cmd_yaw_vel` | `std_msgs/msg/Float64` | 云台 yaw 角速度，rad/s |
| `/<team>/<type>/cmd_pitch_vel` | `std_msgs/msg/Float64` | 云台 pitch 角速度，rad/s |
| `/<team>/<type>/cmd_shoot` | `std_msgs/msg/Bool` | 射击开关 |

四个指令都是**持续生效**的，不会超时自动清零，必须显式发送零值或`false`。

### 3.2 状态反馈（机器人 → 选手）

| 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- |
| `/<team>/<type>/feedback_yaw_vel` | `std_msgs/msg/Float64` | 100 Hz | 云台 yaw 实际角速度 |
| `/<team>/<type>/feedback_pitch_vel` | `std_msgs/msg/Float64` | 100 Hz | 云台 pitch 实际角速度 |
| `/<team>/<type>/feedback_yaw_angle` | `std_msgs/msg/Float64` | 100 Hz | 云台 yaw 单圈角度，`[-π, π]` |
| `/<team>/<type>/feedback_pitch_angle` | `std_msgs/msg/Float64` | 100 Hz | 云台 pitch 单圈角度，`[-π, π]` |
| `/<team>/<type>/chassis_odometry` | `nav_msgs/msg/Odometry` | 100 Hz | 平面真值里程计 |
| `/<team>/<type>/camera/image` | `sensor_msgs/msg/Image` | 配置 30 Hz | 第一视角图像，`rgb8` |
| `/<team>/<type>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 配置 30 Hz | 相机内参，`plumb_bob` |
| `/<team>/<type>/gimbal_imu` | `sensor_msgs/msg/Imu` | 配置 200 Hz | 云台 IMU |
| `/<team>/sentry/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 配置 20 Hz | 哨兵点云，仅哨兵有 |

### 3.3 裁判与比赛（裁判 → 选手）

| 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- |
| `/referee_system/<robot_name>/status` | `msg/RobotStatus` | 10 Hz | 单台机器人的血量、热量、发弹、存活，以及复活读条 / 无敌 / 虚弱状态 |
| `/referee_system/match/status` | `msg/MatchStatus` | 10 Hz | 比赛阶段与已进行秒数 |


### 3.4 浏览器输入

| 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- |
| `/red/infantry/player_input`、`/blue/infantry/player_input` | `msg/PlayerInput` | 60 Hz | 浏览器发布的原始键鼠状态 |

## 4. ROS 域介绍

### 4.1 域划分

| 域 | 用途 |
| --- | --- |
| 内部域（取决于`ROS_DOMAIN_ID`） | 选手不应使用 |
| `20` | 红方步兵 |
| `21` | 红方哨兵 |
| `30` | 蓝方步兵 |
| `31` | 蓝方哨兵 |

启动时内部域**必须不同于 20/21/30/31**，否则启动直接报错。

### 4.2 切换域

```bash
# 当前 shell 长期切换
export ROS_DOMAIN_ID=20

# 只让一条命令生效
ROS_DOMAIN_ID=20 ros2 topic echo /red/infantry/feedback_yaw_angle
```
