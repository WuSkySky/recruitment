# Gazebo Classic 11 迁移基线

记录正式项目切换到 Gazebo Classic 11 时的接口、ROS 域划分和关键配置，作为迁移与验收的
对照基线。迁移前的参考提交为 `f0a2987`（见 `classic-baseline-commit.txt`）。

- 平台：Ubuntu 22.04、ROS 2 Humble、**Gazebo Classic 11.10.2**、gazebo_ros_pkgs 3.9.0。
- 后端：只保留 Gazebo Classic，不提供 Fortress 双后端；参考工作空间 `v2/rmu_ws` 不修改。
- Gazebo Classic 上游已于 2025-01 结束维护，依赖按当前已安装版本固定，不做版本升级。

## ROS 域划分

| 域 | 用途 | `ROS_DOMAIN_ID` |
| --- | --- | --- |
| 内部域 | 裁判、仿真生命周期、插件控制、小消息网关、Web 后端 | 进程环境变量（默认由启动者指定，必须不等于 20/21/30/31） |
| 红方步兵 | 选手话题与传感器 | 20 |
| 红方哨兵 | 选手话题与传感器 | 21 |
| 蓝方步兵 | 选手话题与传感器 | 30 |
| 蓝方哨兵 | 选手话题与传感器 | 31 |

域号在 `recruitment_sim_bringup/launch/bringup.launch.py` 的 `ROBOT_DOMAIN_IDS` 中固定。
启动时若内部域与任一机器人域重合，`generate_launch_description` 直接报错退出。

## 选手接口（机器人域，`/<color>/<type>`）

迁移前后保持一致，话题名、消息类型、命名空间均未改变。

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 订阅 | `cmd_chassis_vel` | `geometry_msgs/msg/Twist` |
| 订阅 | `cmd_yaw_vel`、`cmd_pitch_vel` | `std_msgs/msg/Float64` |
| 订阅 | `cmd_shoot` | `std_msgs/msg/Bool` |
| 发布 | `feedback_yaw_vel`、`feedback_pitch_vel`、`feedback_yaw_angle`、`feedback_pitch_angle` | `std_msgs/msg/Float64` |
| 发布 | `chassis_odometry`（平面真值） | `nav_msgs/msg/Odometry` |
| 发布 | `camera/image`、`camera/camera_info` | `sensor_msgs/msg/Image`、`CameraInfo` |
| 发布 | `gimbal_imu` | `sensor_msgs/msg/Imu` |
| 发布 | `livox/lidar`（仅哨兵） | `sensor_msgs/msg/PointCloud2` |
| 发布 | `player_input`（仅步兵，由 Web 端产生） | `recruitment_sim_interfaces/msg/PlayerInput` |
| 发布 | `/referee_system/<robot_name>/status` | `recruitment_sim_interfaces/msg/RobotStatus` |
| 发布 | `/referee_system/match/status` | `recruitment_sim_interfaces/msg/MatchStatus` |
| 发布 | `/clock` | `rosgraph_msgs/msg/Clock` |

`chassis_odometry` 始终是机器人域中的平面真值输出；`use_odometry` 只控制内部域
`/<color>/<type>/robot_base/odom` 是否发布累计噪声里程计。这一区分在迁移前后相同。

### 底盘与云台控制

- 底盘沿用施力 + PID：`chassis_->AddRelativeForce/AddRelativeTorque`，速度/角速度 PID
  参数为 `(100,0,0)`、`(500,0,0)`、`(200,0,0)`，与迁移前一致。
- 云台走 Classic 的 ODE 关节电机：`Load()` 中 `SetParam("fmax", 0, 20.0)` 给出力矩上限，
  `Update()` 中用 `SetParam("vel", 0, ...)` 下发目标关节速度。**不要用
  `Joint::SetVelocity`** —— ODE 下它等价于 `SetVelocityMaximal`，直接改写子链接速度，
  既压不住重力也无法稳定跟随指令。
- 速度指令会积分成目标角，并叠加位置 PI（`8*误差 + 2*积分`，积分限幅 ±0.5，速度限幅
  ±10 rad/s），对应 Fortress 版本 `JointController` 的 `p_gain`/`i_gain`，用来消除 ODE
  速度电机软约束在重力下产生的稳态爬行。
- 灯条：`robots.yaml` 的颜色在启动时烘焙进机器人 SDF 的 `light_bar_visual` 材质，
  运行期的 `SetLightColor`/enabled 复位仍通过 `~/visual` 材质消息生效。

## 内部接口（仅内部域，不进入选手网关）

| 话题/服务 | 类型 | 说明 |
| --- | --- | --- |
| `/referee_system/match/info` | `MatchInfo` | 完整比赛信息，10 Hz、可靠、transient-local |
| `/referee_system/match/status` | `MatchStatus` | 阶段与已进行时间，转发到四个选手域 |
| `/referee_system/match/control` | `ControlMatch` 服务 | `0` 开始、`1` 结束、`2` 重置并恢复 |
| `/referee_system/simulation/frame` | `SimulationFrame` | WorldPlugin 汇总的帧：轮次、时间、暂停/就绪、机器人位置、有序发弹/命中事件 |
| `/referee_system/simulation/control` | `ControlSimulation` 服务 | WorldPlugin 生命周期操作：`CONFIG`/`PAUSE`/`RESET`/`RESUME`，带 token 去重 |
| `/referee_system/<robot_name>/set_enabled` | `SetRobotEnabled` 服务 | 整机/底盘/云台/发射机构失能 |
| `/referee_system/<robot_name>/reset` | `ResetRobot` 服务 | 回位、清命令、恢复灯条颜色 |
| `/referee_system/<robot_name>/initialize_odometry` | `InitializeModule` 服务 | 清空累计里程计 |
| `/<color>/<type>/robot_base/set_light_color` | `SetLightColor` 服务 | 运行期灯条改色，暂停时同样生效 |
| `/<color>/<type>/robot_base/odom` | `Odometry` | 可选累计噪声里程计（`use_odometry`） |

原 Fortress 版本的 V1 字符串协议已删除，`SimulationFrame`/`ControlSimulation` 只在内部域使用。

## 分域小消息网关白名单

`recruitment_sim_bringup/src/team_topic_bridge.cpp` 每方一个进程，按命名空间逐一转发：

- 选手域 → 内部域：`cmd_chassis_vel`、`cmd_shoot`、`cmd_yaw_vel`、`cmd_pitch_vel`。
- 内部域 → 选手域：`feedback_yaw_vel`、`feedback_pitch_vel`、`feedback_yaw_angle`、
  `feedback_pitch_angle`、`/referee_system/<robot_name>/status`，以及步兵的 `player_input`。
- 内部域 → 选手域（公共）：`/clock`（best-effort，深度 1）、`/referee_system/match/status`
  （transient-local，深度 1）。

网关只承载小消息；图像、点云、IMU 等大数据不经过网关，也没有 `ros_gz_bridge` 或 Ignition 话题映射。

## 传感器与 QoS

- 相机、IMU、GPU 雷达使用 Classic 传感器插件（`RecruitmentSimSensors`）在各自机器人域直接发布，
  没有中间 ROS 桥。
- 发布端 QoS 为 `rclcpp::QoS(2).reliable()`，与可靠订阅端（Web/RViz）和 best-effort 订阅端
  （常见选手代码）都兼容。
- 图像编码 `rgb8`，步兵 `640x480`、哨兵 `1920x1080`；`camera_info` 使用针孔模型，由 HFOV 推导。
- 雷达使用 Classic `gpu_ray`，点云输出 `32` 行、`x/y/z/intensity` 四个 `float32` 字段，
  `row_step = point_step * width`。
- 雷达传感器必须保持 `<visualize>false</visualize>`。Classic 的 gzclient 会把设为 `true`
  的射线传感器逐条画成线段，`1500 × 32` 条、10 Hz、两台哨兵会让整个场景被蓝色射线网格覆盖。
  该开关只影响 GUI 可视化，不改变点云、频率、量程或噪声。

## 关键配置

`config/robots.yaml`（默认）：红蓝双方各一台步兵、一台哨兵；红方出生区
`x∈[-6.0,-4.5]`、`y∈[2.0,4.0]`，蓝方与之中心对称；四台机器人的 `referee` 参数
（`max_hp`/`heat_limit`/`cooling_rate`）和 `projectile_noise` 方差。
中央控制区边界 `[-1.5, 1.5, -1.5, 1.5]`。

`config/base_params.yaml`：`world_name`、`use_odometry`（默认 `false`），以及
`actuator_noise` / `sensor_noise` 各分量的方差，默认全为 `0.0`。启动时逐项注入到
展平后的 SDF 插件参数中。

世界：默认 `rmul_2026h_world.sdf`；`empty_world.sdf` 保留，空场自动关闭占点检测。

## 构建方法

Classic 迁移使用独立的构建目录，避免加载旧 Fortress 插件：

```bash
cd /home/skysky/workspaces/recruitment/v2/recruitment_sim
rm -rf build install                             # 删除 Fortress 时期的构建产物
python3 -m pip install -r requirements.txt      # xmacro、aiohttp、aiortc、av
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --build-base build-classic --install-base install-classic \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install-classic/setup.bash
```

`build-classic/`、`install-classic/` 已在 `.gitignore` 中忽略。旧 `build/`、`install/` 中的
Fortress 插件与 Classic 不兼容，必须删除，不要与新目录混用。

## Classic 资源路径

Fortress 版本的 env hook 只导出 `IGN_*` 变量，source 那个目录会留下空的
`GAZEBO_MODEL_PATH`。此时 Classic 找不到本地 `model://` 资源，会去
`http://models.gazebosim.org/` 拉取模型库并长时间卡住（`ModelDatabase.cc:340`），表现为
“启动卡死”。此外，若 `GAZEBO_RESOURCE_PATH` 不含 `/usr/share/gazebo-11`，Classic 找不到
`media/rtshaderlib`，OGRE 会打印 `Unable to find shader lib` 并退化为无 shader 渲染。

`bringup.launch.py` 因此在**导入 `xmacro4sdf` 之前**就把资源路径写入 `os.environ`：

- `GAZEBO_MODEL_PATH`：本包 `resource/models`，随后追加 `/usr/share/gazebo-*/models`
- `SDF_PATH`：本包 `resource/models`
- `GAZEBO_RESOURCE_PATH`：本包 share 目录，随后追加 `/usr/share/gazebo-*`
- `GAZEBO_PLUGIN_PATH`：本包 `plugins`，随后追加 `/usr/lib/*/gazebo-*/plugins`
- `GAZEBO_MODEL_DATABASE_URI`：未设置时置空，禁用在线模型库

顺序很关键：`xmacro4sdf` 在 import 时读取 `GAZEBO_MODEL_PATH` 来解析
`model://.../model.sdf.xmacro`，如果在 import 之后再注入，机器人 XMacro 展开会先失败。
禁用在线模型库后，真正缺失的模型会立即报 `Unable to find uri[...]`，而不是静默等待网络。

## 依赖边界

- 直接依赖：`gazebo`（Classic 11）、`gazebo_ros`、`sdformat9`、`rclcpp`/`rclpy`、
  `ignition-math6`（Gazebo Classic 自带）。
- 项目源码、`package.xml`、`CMakeLists.txt` 中没有任何 `ros_gz`、`ignition-gazebo`、
  `gz-transport`、`gz-sim` 引用，也没有 `ros_gz_bridge` 进程。
- 插件 `ldd` 结果里出现的 `libignition-transport8`、`libignition-msgs5` 是
  `libgazebo_transport.so.11` 自身的传递依赖，不是项目的直接依赖，也不代表运行 Ignition Gazebo。
- 系统里仍安装有 `ros-humble-ros-gz*` 包，但正式项目不引用、不启动它们。
