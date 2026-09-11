# Gazebo Classic 11 迁移验收记录

对照 `classic-baseline.md` 的接口与域基线，在 Gazebo Classic 11.10.2 / ROS 2 Humble /
Ubuntu 22.04 上完成的构建、测试与运行验收。所有命令均在
`/home/skysky/workspaces/recruitment/v2/recruitment_sim` 下执行，使用独立的
`build-classic` / `install-classic` 目录。

## 1. 干净全量构建

删除 `build-classic`、`install-classic` 后重新构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --build-base build-classic --install-base install-classic \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```

结果：`Summary: 6 packages finished [1min 46s]`，退出码 `0`。仅
`recruitment_sim_description` 有一处 Gazebo 自带 `FindPkgConfig` 的 CMake 开发警告
（`find_package_handle_standard_args` 名称不匹配），不影响构建。

确认不再依赖 Ignition Gazebo / Transport / Messages：

- `src/` 的 `package.xml`、`CMakeLists.txt`、`setup.py` 中没有任何
  `ros_gz`、`ignition-gazebo`、`ignition-transport`、`ignition-msgs`、`gz-sim`、`gz-transport` 引用。
- 运行时没有 `ros_gz_bridge` / `parameter_bridge` / Ignition 进程。
- `ldd install-classic/recruitment_sim_description/plugins/*.so` 只直接链接 Gazebo Classic
  （`libgazebo_*.so.11`）与 `libignition-math6`；出现的 `libignition-transport8`、
  `libignition-msgs5` 由 `libgazebo_transport.so.11` 传递引入，不属于项目直接依赖。

## 2. 单元测试

```bash
colcon test --base-paths src --build-base build-classic --install-base install-classic \
  --executor sequential
colcon test-result --test-result-base build-classic --verbose
```

结果：`Summary: 72 tests, 0 errors, 0 failures, 0 skipped`。覆盖裁判规则
（`test_referee_engine`、`test_match_engine`）、基础控制算法（使能状态、高斯噪声、
增量里程计、初始化门控）、射击方向噪声和 Web 协议。

前端（Vue 输入、导航、裁判页面）：

```bash
cd src/recruitment_sim_env/recruitment_sim_player_web/frontend && npm test
```

结果：`Test Files 3 passed (3)`、`Tests 9 passed (9)`。

原先依赖 Fortress 消息的 `test_light_bar_controller.cpp` 已删除；灯条行为改由
`classic_acceptance.py` 在暂停状态下调用 `SetLightColor` 覆盖（每次重置都验证）。

## 3. 默认四机器人集成验收

```bash
# 终端 A
GAZEBO_MASTER_URI=http://127.0.0.1:11346 ROS_DOMAIN_ID=101 \
  ros2 launch recruitment_sim_bringup bringup.launch.py gui:=false player_web_port:=8082
# 终端 B
source install-classic/setup.bash
ROS_DOMAIN_ID=101 python3 src/recruitment_sim_env/recruitment_sim_bringup/test/classic_acceptance.py --resets 20
```

结果：20/20 次 `READY → RUNNING → FINISHED` 全部通过，退出码 `0`。最后一条命令同时验证：

- 四个域的相机、IMU、里程计全部有数据；两台哨兵的点云为 32 行结构。
- 步兵相机 `640x480 rgb8`、哨兵相机 `1920x1080 rgb8`，帧内像素非恒定。
- 底盘按 `cmd_chassis_vel` 实际移动，云台按 `cmd_yaw_vel` 实际转动并回传
  `feedback_yaw_angle`。
- `cmd_shoot` 触发真实发弹（`RobotStatus.total_shots` 增加）。
- `SetRobotEnabled(target=3, enabled=false)` 后再次扣扳机不再记录发弹。
- 每次重置后 `total_shots == 0`，且所有域传感器计数继续增长；暂停期间灯条改色成功。

最后一次运行的收发计数（节选）：

| 键 | 计数 |
| --- | --- |
| `frame` | 7881 |
| `red/infantry/image` | 116 |
| `red/infantry/imu` | 1755 |
| `red/infantry/odom` | 8662 |
| `red/sentry/lidar` | 108 |
| `blue/sentry/lidar` | 109 |
| `match` | 260 |
| `status` | 260 |

## 4. 单机器人启动

使用只含一台红方步兵的配置：

```bash
ros2 launch recruitment_sim_bringup bringup.launch.py gui:=false \
  robots_file:=/path/to/robots_single.yaml
ROS_DOMAIN_ID=101 python3 .../test/classic_acceptance.py --single --resets 5
```

结果：启动无错误，内部域节点为 `/gazebo`、`/red/infantry/robot_base`、
`/red_infantry_topic_bridge`、`/referee_system`、`/referee_system/simulation`；5/5 次
`READY → RUNNING → FINISHED` 通过，红方步兵域相机、IMU、里程计持续输出。

比赛终端需要红蓝双方各一台步兵，因此单机器人等不完整阵容下启动会打印
`player_web skipped: ...` 并跳过 Web 节点，仿真其余部分正常运行（此前会直接抛出
`RuntimeError` 并留下错误堆栈）。需要完整 Web 终端时使用默认四机器人配置。

## 5. 分域隔离与网关边界

逐域列出话题与服务：

| 域 | 话题（节选） | 服务 |
| --- | --- | --- |
| 101 内部 | `/clock`、`/referee_system/match/info`、`/referee_system/match/status`、四个 `.../cmd_*`、四个 `.../status` | `/referee_system/match/control`、`/referee_system/simulation/control`、`.../set_enabled`、`.../reset`、`.../initialize_odometry`、`*/robot_base/set_light_color` |
| 20 红步兵 | `/red/infantry/{camera/image,camera/camera_info,gimbal_imu,chassis_odometry,cmd_*,feedback_*,player_input}`、`/clock`、`/referee_system/{match/status,red_infantry_robot/status}` | 仅参数服务 |
| 21 红哨兵 | 同 20，另有 `/red/sentry/livox/lidar` | 仅参数服务 |
| 30 蓝步兵 | `/blue/infantry/...`，无任何红方话题 | 仅参数服务 |
| 31 蓝哨兵 | `/blue/sentry/...`，含 `/blue/sentry/livox/lidar` | 仅参数服务 |

结论：

- 每个机器人域只含自己的话题，跨队不串扰。
- `/referee_system/match/info`、`/referee_system/simulation/frame`、
  `/referee_system/match/control`、`/referee_system/simulation/control` 以及各
  `set_enabled`/`reset`/`initialize_odometry` 服务只存在于内部域，选手域中不可发现。
  这一条也已写进 `classic_acceptance.py` 的断言。
- `/clock` 由 Classic 通过 `gazebo_ros_init` 在内部域发布，再由 `team_topic_bridge`
  以 best-effort 转发到四个选手域。
- 图像、`camera_info`、IMU、点云都由传感器插件在各自机器人域直接发布，没有任何桥接进程，
  也没有大数据 ROS 转发链路。
- 内部域中出现的 `/tf`、`/tf_static`、`/robot_description` 只是 RViz 的订阅端条目
  （`Publisher count: 0`），项目不发布 TF、`joint_states` 或 `robot_description`。

## 6. GUI 与 RViz

```bash
QT_QPA_PLATFORM=xcb ros2 launch recruitment_sim_bringup bringup.launch.py \
  gui:=true rviz:=true player_web_port:=8082
```

- `gzclient` 11.10.2 连接 master 成功，创建 `2880x1518` 主窗口和 `2572x1341` 渲染视口，
  状态栏显示 `Sim Time` 持续推进、`Real Time Factor ≈ 0.94`、`FPS ≈ 20`。
- `gzclient` 唯一告警是 `InsertModelWidget` 对若干只作为 XMacro 定义、没有独立
  `model.config` 的模块报 `Missing model.config`；这些模块由机器人模型内联展开，不影响
  场地和机器人渲染，属于历史遗留的 GUI 插入面板提示。
- 场地视觉与碰撞通过机器人相机和雷达数据交叉核对：相机可见分层地面（浅灰）、高地
  （灰蓝）和围墙（深灰）；雷达点云在 16 m 视场内形成连续的墙面与高地轮廓。
- `rviz2` 正常启动，加载 `visualize_robot.rviz`（Grid、RobotModel、TF），
  `OpenGl version: 4.6`，网格以 28 fps 渲染。
- RViz 的 RobotModel/TF 显示没有数据，因为项目按约定不发布 `robot_description`、
  TF 和 `joint_states`。这一点与迁移前一致：迁移前（`f0a2987`）的
  `ros_gz_bridge` 映射表里同样没有 `/tf` 或 `/joint_states`，也没有
  `robot_state_publisher`；当时只有 Ignition 侧未被桥接的
  `ignition-gazebo-joint-state-publisher-system`。

环境的限制：当前主机是 GNOME/Wayland，合成器禁止截图
（`org.gnome.Shell.Screenshot` 返回 `AccessDenied`），且 Gazebo 的 GL 子窗口无法通过
`XGetImage` 取回内容，因此没有保存 GUI 截图。GUI 的验证依据是窗口/视口尺寸、渲染循环
FPS、Sim Time 推进、日志无渲染错误，以及相机/雷达数据与场地几何互相印证。

## 7. Web 比赛终端

```bash
ROS_DOMAIN_ID=101 python3 src/recruitment_sim_env/recruitment_sim_bringup/test/web_acceptance.py --port 8082
```

结果：`SUMMARY: 14/14 checks passed`，退出码 `0`。覆盖：

- `GET /`、`/referee`、`/player/red`、`/player/blue` 返回 200。
- `GET /api/roles` 返回 `red`/`blue`/`referee` 与 `referee_online=true`。
- 裁判页 `WS /ws/referee` 收到 `status` 流（含比赛状态与四台机器人）。
- 裁判状态机：从 `RUNNING` 依次执行 `end → FINISHED → reset → READY`，每一步都被接受；
  从 `TRAINING`/`FINISHED`/`ERROR` 直接 `reset` 也会进入 `READY`。
- `WS /ws/red` 返回 `ready` 且 `player_robot=red_infantry_robot`；同一角色第二次连接被拒
  （`occupied`）。
- `ping`/`pong` 往返正常。
- 发送 `{"type":"input","pressed_keys":["KeyW","MouseLeft"],"mouse_dx":12.5}` 后，
  红方步兵域收到 `player_input`，`pressed_keys` 与 `mouse_dx` 一致——键鼠链路由 Web 端
  经小消息网关到达选手域。
- `POST /api/webrtc/offer` 返回带 `m=video` 的 answer，并成功解码出一帧 `640x480` 视频。
- 最后在 `READY` 状态发送 `start` 并被接受。

相机只在仿真运行时出帧，因此脚本会先按裁判页允许的状态机把比赛驱动到 `READY` 再测视频；
这一点在暂停（`FINISHED`）和运行中（`RUNNING`）两种初始状态下都验证过。

## 8. 传感器数据抽样

```bash
ROS_DOMAIN_ID=101 python3 src/recruitment_sim_env/recruitment_sim_bringup/test/sensor_probe.py out.png
```

- 相机：`640x480 rgb8`，`frame_id=front_industrial_camera_optical_frame`，三通道动态范围
  约 130（画面非空白）。
- IMU：`frame_id=red_infantry_robot/gimbal_pitch/gimbal_imu`，四元数模长 `1.000`。
- 雷达：`1875x32` 点、`point_step=16`、`row_step=30000`、`frame_id=front_mid360`，
  60000 点中 57729 点为有限值，测距范围 `[0.00, 13.14] m`，俯视图呈现连续的场地轮廓。

## 9. 迁移期间修正的问题

1. 弹丸删除时机：在遍历接触数据时删除弹丸会触发 Classic 的对象失效断言，改为在下一个
   物理步删除。
2. 暂停切换死锁：ROS 回调与物理更新共用状态锁，重置时改为在调用
   `World::SetPaused` 前后按序释放/重新获取锁；同时跳过 0 自由度关节的速度复位。
3. 点云打包：`PointCloud2Modifier::resize` 会把高度压成 1，重新显式设置 `height` 与
   `row_step`，恢复 32 行结构。
4. 裁判初始握手：Classic 加载世界并创建 `WorldPlugin` 可能超过 10 s，首次 `CONFIG`
   阶段的宽限延长到 60 s（其余阶段仍为 10 s，超时进入 ERROR 的行为不变）。
5. 单机器人启动：比赛终端需要红蓝双方步兵，改为在不完整阵容下跳过 `player_web` 并打印
   提示，而不是抛出异常。
6. 启动卡死（用户报告）：残留的 Fortress 版 `install/` 只导出 `IGN_*` 变量，source 后
   `GAZEBO_MODEL_PATH` 为空，Classic 无法解析本地 `model://` 资源并转去
   `models.gazebosim.org`，永久卡在 `ModelDatabase.cc:340`。修复分三部分：
   - 删除 Fortress 时期构建出的 `build/`、`install/`（本机已删除，仓库内不再存在）。
   - `bringup.launch.py` 在导入 `xmacro4sdf` 之前自行注入
     `GAZEBO_MODEL_PATH`/`SDF_PATH`/`GAZEBO_RESOURCE_PATH`/`GAZEBO_PLUGIN_PATH`
     并置空 `GAZEBO_MODEL_DATABASE_URI`；顺序是必需的，因为 `xmacro4sdf` 在 import 时
     读取 `GAZEBO_MODEL_PATH`。
   - 同时补上 `/usr/share/gazebo-*` 与 `/usr/lib/*/gazebo-*/plugins`，消除此前每次启动都会
     出现的 `Unable to find shader lib`（`GAZEBO_RESOURCE_PATH` 缺失导致 OGRE 关闭
     shader 生成）。

   复现与回归（`env` 中完全没有任何 `GAZEBO_*` 变量，模拟残留 install 的环境）：

   ```bash
   source install-classic/setup.bash
   unset GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH GAZEBO_MODEL_DATABASE_URI
   ros2 launch recruitment_sim_bringup bringup.launch.py gui:=false player_web:=false
   ```

   修复前：日志停在 `Getting models from[http://models.gazebosim.org/]`，无机器人出现。
   修复后：4 台机器人全部生成，`models.gazebosim.org` 访问次数 0，
   `Unable to find shader lib` 0 次，`not find xmacro_include` 0 次。
7. GUI 整个场景铺满蓝色网格（用户报告）：`mid360` 雷达带 `<visualize>true</visualize>`。
   Ignition 的 GUI 不绘制射线，迁移前没有可见影响；Classic 的 gzclient 会把每条射线画成
   蓝色线段，`1500 × 32` 条、10 Hz、两台哨兵足以让整个场景被射线网格覆盖。相机、雷达点云
   等数据本身正常，只是 GUI 可视化。修复：`mid360/model.sdf` 与
   `mid360/model.sdf.xmacro` 改为 `<visualize>false</visualize>`。

   定位过程（可复现）：标准 `empty.world` 和本项目的 `empty_world.sdf` 在 gzclient 中都渲染
   正常；只有 RMUL 场地 + 带雷达的哨兵才会出现蓝色网格，1 台步兵时正常，因此问题定位到
   雷达的射线可视化而不是驱动、GL 版本或相机位姿。
8. 云台速度控制无效并因重力扭动（用户报告）。Classic/ODE 下 `Joint::SetVelocity` 走的是
   `SetVelocityMaximal`，即**直接改写子链接速度**，不是关节电机；实测命令 `1.0 rad/s`
   只得到 `0.044 rad/s`，静止时 pitch 停在 `+0.37 rad`、yaw 在 12 s 内漂移 `0.146 rad`。
   修复分两步：
   - 在 `Load()` 里给云台关节设置 ODE 电机力矩上限（`SetParam("fmax", 0, 20.0)`），
     `Update()` 里用电机目标速度 `SetParam("vel", 0, ...)` 而不是 `SetVelocity`。
   - 把速度指令积分成目标角，叠加位置 PI（对应 Fortress 版本 `JointController` 的
     `p_gain`/`i_gain`）。ODE 速度电机是软约束，纯速度指令在重力力矩下仍有约
     `0.002 rad/s` 的稳态爬行，位置项把它压住、积分项消除残差。

   修复后实测：命令 `1.0 rad/s` → `0.977 rad/s`；静止 yaw 抖动 `0.0005 rad`，
   pitch 不再有单调漂移。`classic_acceptance.py` 的云台断言也从"角度变化 > 0.05 rad"
   （会被漂移蒙混通过）改成"静止漂移 < 0.05 rad"加"跟随速率 > 0.2 rad/s"。
9. 灯条颜色在启动和重置后是白色（用户报告）。`light_bar_visual` 的材质在模型定义里
   硬编码为白色，颜色只靠运行时 Gazebo transport 消息更新；而 Classic 的 GUI 会从模型
   SDF 重建视觉（重置尤其如此），运行时颜色会被覆盖回白色。修复：`bringup.launch.py`
   在生成机器人 SDF 时，把 `robots.yaml` 里的颜色烘焙进该机器人全部
   `light_bar_visual` 的 `ambient`/`diffuse`/`emissive`；运行时的材质消息继续保留，
   用于比赛期间的动态改色。
10. 弹丸互相碰撞销毁、看起来没有回收（用户报告）。订阅 World 插件的事件流可以直接看到

    ```
    HIT id=39 target=red_infantry_robot_projectile_0_40
    HIT id=40 target=red_infantry_robot_projectile_0_39
    ```

    即相邻弹丸在炮口互相命中。根因是 Gazebo Classic 的 `World::processMsgsPeriod` 固定为
    **200 ms（墙钟）**，模型插入消息每 200 ms 才处理一次；而射击间隔是 50 ms，于是 3–4 发
    弹丸会在**同一个仿真时刻、同一个炮口位姿**被创建，彼此重叠后在第一帧就互相碰撞。
    原始 Fortress 实现有一条 `spawnedProjectiles.empty() || back().isInit` 的约束来规避，
    迁移时丢失。修复：恢复该约束，`launcher_ready = projectiles_.empty() || back().spawned`，
    即上一枚弹丸真正生成后才允许发下一枚。

    修复后实测：射击间隔稳定在 ~190 ms（受 200 ms 消息周期限制），每一发都命中
    `RMUL_2026` 场地后被回收；开火中世界内 2 枚弹丸、停火 2.5 s 后归零、重置后
    `get_model_list` 只剩 `RMUL_2026` 与 4 台机器人。
    **副作用**：实际射速上限约 5 发/秒，低于原设计的 20 发/秒；这是 Classic 实体插入
    周期的平台限制，若要恢复 20 发/秒需要改为预创建弹丸池并复用实体。
11. 移除默认世界的通用 `ground_plane`。`rmul_2026h_world.sdf` 原本沿用参考工程的
    100 × 100 m 静态平面（浅灰可视 + `mu=100/mu2=50` 摩擦），而场地模型 `RMUL_2026`
    自带完整碰撞网格 `RMUL_2026H.stl`（含地面），机器人实际踩在后者上。

    对照实测（单机器人，其余配置不变）：

    | 项目 | 带 `ground_plane` | 去掉 `ground_plane` |
    | --- | --- | --- |
    | 静止高度 z | 0.076798 | 0.076798 |
    | 底盘位置漂移 | -5.2674 → -5.2692 | -5.2674 → -5.2693 |
    | 直线行驶（0.5 m/s，2 s） | — | 0.996 m |

    去掉后 4 台机器人静止高度仍全部为 0.076798，世界模型列表只剩 `RMUL_2026` 与
    4 台机器人，`classic_acceptance --resets 5` 全部通过。
    `empty_world.sdf` 保留 `ground_plane`——那里它是唯一的地面，删除会导致机器人下坠。

## 10. 性能测量

平台 16 核；测量时系统 load average 约 3–4.6（桌面与浏览器占用约一个核），
因此绝对值会随后台负载波动，只有同一时段的横向比较有意义。

| 场景 | 稳态 RTF |
| --- | --- |
| 单机器人 · 无界面 | **0.9994** |
| 4 机器人 · 无界面 · 空闲 | **0.976**（里程计降频前为 0.955） |
| 4 机器人 · 带界面 · 空闲 | **0.969** |
| 4 机器人 · 带界面 · 满载 | **0.670** |

单机器人（无界面）实测速率：相机 `30.0 Hz`（配置 30）、IMU `200 Hz`、云台反馈 `100 Hz`、
里程计 `998 Hz`（降频前）。

4 机器人（无界面 · 空闲）实测速率：相机 `10.9 Hz`（配置 30）、雷达 `10.9 Hz`（配置 10）、
IMU `187–194 Hz`、里程计与云台反馈 `98 Hz`；CPU 为 gzserver ≈ 1.7–1.9 核、gzclient ≈ 0.7 核。

结论：

1. 单机器人时全部指标达标，说明物理步进与控制链路没有瓶颈。
2. 多机器人时瓶颈是**传感器渲染**：4 路相机（2×640×480、2×1920×1080）加 2 路 GPU 雷达
   把渲染循环压到约 11 Hz，相机达不到配置的 30 Hz，RTF 随之下降。这是 gzserver 内按步
   串行的渲染开销，增加核心数无效；提升只能靠降低相机分辨率/频率或减少相机数量，
   而这些受"不改变相机分辨率与传感器频率"的约束，本次未做改动。
3. "满载"是人为构造的极端负载：4 台车同时全速运动、云台扫掠、无限开火。真实比赛里
   裁判的热量规则会限制持续射击，实际负载介于空闲与满载之间。
4. 车辆顶到墙边时相机会输出纯色画面，验收脚本的"画面非空"断言会因此失败；请在车辆
   位于场地正常位置时运行验收。

### 里程计降频

迁移后的 `chassis_odometry` 原本每个物理步都发布（单机器人实测 998 Hz、4 机器人约 670 Hz，
合计约 2700 msg/s）。这与迁移前 `MecanumDrive2::UpdateOdometry` 的逐步发布行为一致，并非
迁移引入的回归，但消费端（选手、Web、RViz）并不需要该速率，因此改为 **100 Hz**，与云台
反馈一致。效果：空闲 RTF 由 0.955 提升到 0.976；满载 RTF 不变（0.703），因为满载瓶颈在
渲染而非消息速率。

## 11. 复现方式

`recruitment_sim_bringup/test/` 下提供三个手动验收脚本，都需要先启动 bringup 并在同一
内部域运行：

- `classic_acceptance.py`：四机器人或 `--single` 单机器人的接口、控制、射击、失能与
  连续重置验收。
- `web_acceptance.py`：Web 静态页、角色互斥、键鼠输入、WebRTC 与裁判控制链路验收。
- `sensor_probe.py`：相机、IMU、雷达数据抽样与俯视可视化。

它们不注册进 `colcon test`，因为需要一个正在运行的仿真；自动化单元测试仍由
`colcon test` 覆盖。
