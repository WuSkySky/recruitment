# Recruitment Simulation 开发上下文

## 项目定位

- `/home/skysky/workspaces/recruitment/v2/recruitment_sim` 是当前正式开发项目。
- 这是一个面向招新选拔的 ROS 2 / Gazebo 仿真比赛环境。项目方负责提供场地、机器人模型、机器人基础控制和裁判系统；参赛选手在这些基础设施之上实现感知、定位、导航、决策等高级功能，并通过完成比赛任务参加选拔。
- `/home/skysky/workspaces/recruitment/v2/rmu_ws` 是开源参考仿真工作空间。后续会从中继续选择性移植模型、插件和裁判相关逻辑。除非任务明确要求，不要直接修改该参考工作空间。
- 从参考项目迁移内容时，应保留原版权头、许可证和第三方来源说明。迁移不是简单整仓复制：只引入当前项目需要的内容，并按本项目包名、命名空间和接口边界完成适配。

## 运行环境与目录结构

- 目标平台：Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress（Ignition Gazebo 6）。
- ROS 工作空间源码位于 `src/recruitment_sim_env/`。
- 工作空间固定为四个自有功能包：
  - `recruitment_sim_interfaces`：自有消息和服务接口。
  - `recruitment_sim_description`：机器人与场地描述、模型资源、SDF→URDF 工具和 Gazebo 插件。
  - `recruitment_sim_robot_base`：底盘、云台、射击、灯条和里程计等基础控制。
  - `recruitment_sim_bringup`：场景启动、机器人生成、ROS–Gazebo 桥接、TF 和 RViz。
- 不要重新拆出原先设想的模型、插件或转换工具包；机器人描述与插件目前统一归入 `recruitment_sim_description`。

## 当前进度（2026-09-06）

- 已迁移 `pb2025_infantry_robot` 和 `pb2025_sentry_robot` 的完整基础仿真功能，而非只有外观模型。
- 步兵已包含工业相机；哨兵已包含工业相机、Mid360 点云和 IMU。
- 已具备麦轮底盘、云台控制、射击、灯条切换、关节状态、TF、传感器桥接以及可选里程计；每台机器人通过独立 ROS 命名空间隔离。
- 已提供 YAML 机器人配置，可启动单台、多台或多个同类型机器人。默认启动红色步兵 `infantry_robot` 和蓝色哨兵 `sentry_robot`。
- 已加入 RMUL 2026 3V3 场地，并作为默认世界；同时保留空场世界。
- 场地中两个高地已经按图纸修正：主体尺寸为 `2.0 × 5.0 × 0.2 m`，内侧挡条为 `0.15 × 1.8 × 0.2 m`，挡条安装后总高度为 `0.4 m`，两侧中心对称。
- 场地围墙使用独立可视网格，四面墙法线朝向场内：从场内可见，从场外因背面剔除而不可见。碰撞网格仍保持完整，因此外部不可见不代表碰撞消失。
- 场地已经按 2026 RMUL 规则手册图示分层配色：深灰围墙、浅灰地面、灰蓝高地、深灰启动/补给区和蓝灰中央控制区。启动/补给区为 `1.5 × 2.0 m`，中央控制区为 `3.0 × 3.0 m`；这些区域目前是可视标识，不额外改变地面碰撞。
- 包名已从 `recruitment_sim_robot` 改为 `recruitment_sim_description`，场地资源也已放入该包。
- 四包全量构建、机器人 XMacro 展开、SDF 合法性检查和无界面双机器人启动此前均已通过；修改场地网格后也已重新通过 `recruitment_sim_description` 构建和 SDF 检查。

## 裁判系统现状与后续方向

- 当前版本有意不启动、不依赖裁判系统，也不发布 `/referee_system/*`。这是为了后续进行较大幅度的裁判系统重构，不代表参考项目的裁判逻辑没有价值。
- 当前仍保留通用仿真能力：弹丸生成、初速度设置、物理碰撞检测，以及碰撞或超时后的自动回收。
- 当前未实现的裁判职责包括有效装甲命中判定、伤害与血量、弹量和供电限制、比赛状态、射击者归属、比赛规则相关的场地判定等。
- 后续重构裁判系统前，应先审计 `rmu_ws` 中已有实现，区分“可复用的通用物理/事件检测”和“需要重写的比赛规则”。不要在未确认复用价值前删除大型裁判逻辑。
- 建议将物理事件采集与比赛规则解耦：模型和插件负责提供命中对象、射击者、弹速、位置等原始事实，新裁判模块负责判定有效性并更新比赛状态。

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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch recruitment_sim_bringup bringup.launch.py gui:=false
```

- 用户接口、启动方法和第三方来源分别以 `README.md`、`THIRD_PARTY_NOTICES.md` 和 `LICENSES/` 为准；相关实现变化后应同步更新这些文档。
