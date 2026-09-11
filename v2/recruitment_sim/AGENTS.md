# Recruitment Simulation 开发上下文

## 项目定位

- `/home/skysky/workspaces/recruitment/v2/recruitment_sim` 是当前正式开发项目。
- 这是一个面向招新选拔的 ROS 2 / Gazebo 仿真比赛环境。项目方负责提供场地、机器人模型、机器人基础控制和裁判系统；参赛选手在这些基础设施之上实现感知、定位、导航、决策等高级功能，并通过完成比赛任务参加选拔。
- `/home/skysky/workspaces/recruitment/v2/rmu_ws` 是开源参考仿真工作空间。后续会从中继续选择性移植模型、插件和裁判相关逻辑。除非任务明确要求，不要直接修改该参考工作空间。
- 从参考项目迁移内容时，应保留原版权头、许可证和第三方来源说明。迁移不是简单整仓复制：只引入当前项目需要的内容，并按本项目包名、命名空间和接口边界完成适配。

## 运行环境与目录结构

- 目标平台：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11.10.2。
- ROS 工作空间源码位于 `src/recruitment_sim_env/`。
- 工作空间包含六个自有功能包：
  - `recruitment_sim_interfaces`：自有消息和服务接口。
  - `recruitment_sim_description`：机器人与场地描述、模型资源、SDF→URDF 工具和 Gazebo 插件。
  - `recruitment_sim_robot_base`：底盘、云台、射击、灯条和里程计等基础控制算法库与测试工具，由机器人插件调用，不注册独立节点。
  - `recruitment_sim_referee_system`：机器人血量、射击热量、命中判定和失能控制。
  - `recruitment_sim_bringup`：场景启动、机器人生成、ROS 分域小消息网关和 RViz。
- 不要重新拆出原先设想的模型、插件或转换工具包；机器人描述与插件目前统一归入 `recruitment_sim_description`。

## 当前进度（2026-09-07）

- 已迁移 `pb2025_infantry_robot` 和 `pb2025_sentry_robot` 的完整基础仿真功能，而非只有外观模型。
- 步兵已包含工业相机；哨兵已包含工业相机、Mid360 点云和 IMU。
- 已具备麦轮底盘、云台控制、射击、灯条切换、传感器直接分域发布以及可选里程计；每台机器人通过独立 ROS 命名空间隔离。ROS 侧不发布 `TF`、`joint_states` 或 `robot_description`（迁移前的 `ros_gz_bridge` 映射同样没有这些话题），RViz 默认配置里的 RobotModel/TF 显示因而没有数据。
- 已提供 YAML 机器人配置，可启动单台、多台或多个同类型机器人。默认启动红蓝双方各一台步兵和一台哨兵，共四台机器人；每方在己方启动区短边居中、沿长边前后排列，步兵在前、哨兵在后，车头沿长边朝向场地中心，双方位姿中心对称。
- 已加入 RMUL 2026 3V3 场地，并作为默认世界；同时保留空场世界。
- 场地中两个高地已经按图纸修正：主体尺寸为 `2.0 × 5.0 × 0.2 m`，内侧挡条为 `0.15 × 1.8 × 0.2 m`，挡条安装后总高度为 `0.4 m`，两侧中心对称。
- 场地围墙使用独立可视网格，四面墙法线朝向场内：从场内可见，从场外因背面剔除而不可见。碰撞网格仍保持完整，因此外部不可见不代表碰撞消失。
- 场地已经按 2026 RMUL 规则手册图示分层配色：深灰围墙、浅灰地面、灰蓝高地、深灰启动/补给区和蓝灰中央控制区。启动/补给区为 `1.5 × 2.0 m`，中央控制区为 `3.0 × 3.0 m`；这些区域目前是可视标识，不额外改变地面碰撞。
- 包名已从 `recruitment_sim_robot` 改为 `recruitment_sim_description`，场地资源也已放入该包。
- 五包全量构建、机器人 XMacro 展开、SDF 合法性检查和无界面多机器人启动均已通过。

## 裁判系统现状与后续方向

- 已新增独立 `recruitment_sim_referee_system` 包，默认随 bringup 启动；所有裁判通讯使用 `/referee_system` 前缀。
- 完整比赛信息 `/referee_system/match/info`（`MatchInfo`）仅在内部域发布；`/referee_system/match/status`（`MatchStatus`）仅含比赛阶段 `state` 和已进行仿真秒数 `elapsed_seconds`，转发到四个选手域。
- 弹丸插件仅报告实际发弹和首次碰撞事实，中央裁判节点负责有效装甲筛选、敌我判定、扣血、热量与执行器控制。
- 当前按 RMUL 2026 的 17mm 规则实现：每发热量 +10、10Hz 冷却、普通/本局永久过热锁枪、敌方装甲伤害 20、装甲 50ms 检测间隔和战亡整机失能。
- 每台机器人插件提供统一的 `SetRobotEnabled` 服务，可分别控制整机、底盘、云台和发射机构；失能不关闭传感器或反馈。
- 已实现 TRAINING/RESETTING/READY/RUNNING/ENDING/FINISHED/ERROR 比赛状态机、300 秒倒计时、双方 200 初始胜利点、中央控制区先到先得与离区 2 秒保留、占领扣点、战亡扣点，以及胜利点/攻击伤害/剩余血量的胜负结算。
- `RESUME` 使用原地复位并恢复 Gazebo：删除旧弹丸、恢复机器人出生位姿和裁判/执行器状态，但保留 GPU 传感器实体；通过 Classic API 回位并确认弹丸清理完成；完成后进入 READY。`START` 只开始裁判计时和胜利点规则，不执行重置。
- 尚未实现撞击与 42mm 伤害、回血复活、弹量、初速度处罚、准备阶段和 BO 管理。后续规则应继续保持物理事件采集与比赛逻辑解耦。

## 开发约定

- 所有新增机器人实例必须保持话题和 TF 命名空间隔离，避免多机器人串扰。
- 不要主动改变参考机器人的尺寸、惯量、控制参数、相机分辨率或传感器频率；需要调参时应由具体任务明确提出。
- 场地可视网格与碰撞网格用途不同。修改围墙显示效果时不要破坏碰撞；修改高地几何时应同时核对视觉和碰撞尺寸。
- 修改包名、模型名、话题或插件接口时，应同步检查源码、XMacro/SDF、launch、YAML、README、依赖清单和安装规则中的引用。
- 常用验证命令：

```bash
cd /home/skysky/workspaces/recruitment/v2/recruitment_sim
source /opt/ros/humble/setup.bash
colcon list
colcon build --base-paths src --build-base build-classic --install-base install-classic \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install-classic/setup.bash
ros2 launch recruitment_sim_bringup bringup.launch.py gui:=false
```

- 用户接口、启动方法和第三方来源分别以 `README.md`、`THIRD_PARTY_NOTICES.md` 和 `LICENSES/` 为准；相关实现变化后应同步更新这些文档。

## Classic 迁移（2026-09-11）

- 所有 Classic ModelPlugin、WorldPlugin、SensorPlugin 和共享运行库位于 description/plugins/classic。
- robot_base 是基础算法库，不再提供独立进程或 Ignition 转接模块。
- 保留内部域与 20/21/30/31 四个机器人域；传感器通过按域共享的 ROS Context 直接发布。
- 裁判仿真通信使用 SimulationFrame / ControlSimulation，禁止把内部生命周期接口加入选手网关。
- 修改后使用干净的 build-classic / install-classic 目录验证。旧 install 中的 Fortress 插件不兼容。
- Classic 是已结束维护的固定平台，不引入 Fortress 双后端。
- 迁移基线见 `docs/classic-baseline.md`（域划分、选手/内部接口、网关白名单、QoS、构建方法），验收记录见 `docs/classic-verification.md`。
- `recruitment_sim_bringup/test/` 下的 `classic_acceptance.py`、`web_acceptance.py`、`sensor_probe.py` 需要先启动 bringup 才能运行，不注册进 `colcon test`；请在专用验证实例上运行，不要干扰正在进行的比赛。
- 比赛终端需要红蓝双方各一台步兵；不完整阵容下 bringup 会跳过 `player_web` 并打印提示。
- 裁判首次 `CONFIG` 握手宽限为 60 s（Classic 加载世界可能较慢），其余生命周期阶段超时仍为 10 s 并进入 ERROR。
- 启动文件在导入 XMacro 之前自行注入 Classic 资源路径（`GAZEBO_MODEL_PATH`、`SDF_PATH`、`GAZEBO_RESOURCE_PATH`、`GAZEBO_PLUGIN_PATH`）并置空 `GAZEBO_MODEL_DATABASE_URI`，因此不依赖 shell 是否 source 过 Gazebo 的 setup.sh。改动这段逻辑时注意：`xmacro4sdf` 在 import 时读取 `GAZEBO_MODEL_PATH`，注入必须发生在其 import 之前。
- 不要保留 Fortress 时期构建出的 `build`/`install` 目录；source 旧目录会缺失 `GAZEBO_MODEL_PATH`，导致 Gazebo 卡在在线模型库下载。
- `mid360` 雷达的 `<visualize>` 必须保持 `false`：Classic 的 gzclient 会把 `true` 的射线传感器逐条画出来，两台哨兵就会让整个场景铺满蓝色射线网格。改完模型资源后必须重新 `colcon build` 对应包，否则 XMacro 仍会从已安装目录读到旧值。
- 云台必须走 ODE 关节电机：`SetParam("fmax", 0, ...)` + `SetParam("vel", 0, ...)`，**不要用 `Joint::SetVelocity`**（ODE 下等价于直接改写子链接速度，压不住重力也跟不上指令）。速度指令要积分成目标角并叠加位置 PI，用来消除软约束在重力下的稳态爬行。
- 灯条颜色在 `bringup.launch.py` 里烘焙进机器人 SDF 的 `light_bar_visual` 材质：Classic 的 GUI 会从 SDF 重建视觉，只靠运行时材质消息会在启动/重置后回到白色。
- 弹丸发射必须等上一枚真正生成（`projectiles_.empty() || back().spawned`）：Gazebo Classic 每 200 ms（墙钟）才处理一次实体插入（`World::processMsgsPeriod`），按 50 ms 连发会让多枚弹丸在同一仿真时刻、同一炮口位置被创建而互相碰撞销毁。这也意味着实际射速上限约 5 发/秒；要恢复到 20 发/秒必须改成预创建弹丸池并复用实体。
- 内部生命周期服务（`set_enabled`、`reset`）带 `round_id` 代次校验：重置之后再用 `round_id=0` 的手动调用会被判为过期版本。手动测试需要传入当前代次（`/referee_system/simulation/frame` 的 `round_id`），裁判自身总是带代次，因此不受影响。
- 默认世界的通用 `ground_plane` 已移除：场地模型 `RMUL_2026` 自带含地面的碰撞网格 `RMUL_2026H.stl`，机器人踩的是它，实测静止高度与行驶位移都不变。`empty_world.sdf` 必须保留 `ground_plane`，那里它是唯一地面。
- 真值里程计 `chassis_odometry` 与云台反馈按 100 Hz 发布（原先每个物理步都发）。多机器人时的性能瓶颈是传感器渲染而不是消息速率：4 路相机加 2 路 GPU 雷达会把渲染循环压到约 11 Hz，相机达不到配置的 30 Hz。降低相机分辨率或频率属于被约束项，不要自行修改。
