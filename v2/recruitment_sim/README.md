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

- `recruitment_sim_interfaces`：控制消息和灯条服务。
- `recruitment_sim_description`：机器人与 RMUL 2026 场地描述、模型资源、SDF→URDF 工具和 Gazebo 插件。
- `recruitment_sim_robot_base`：底盘、云台、射击、灯条与里程计控制节点。
- `recruitment_sim_bringup`：机器人配置、生成、桥接、TF 和 RViz 启动。

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
`name`、受支持的 `type`、灯条 `color` 以及 `x/y/z/yaw` 初始位姿；可以删除、复制条目以
启动单台、多台或多个同类型机器人。也可以通过 `robots_file:=/absolute/path/robots.yaml`
加载其他配置。

## 控制接口

将下方 `<robot>` 替换为 `infantry_robot` 或 `sentry_robot`：

- `/<robot>/cmd_vel`：`geometry_msgs/msg/Twist` 底盘速度。
- `/<robot>/robot_base/chassis_cmd`：支持速度和跟随云台模式。
- `/<robot>/cmd_gimbal_joint`：`sensor_msgs/msg/JointState` 云台目标角。
- `/<robot>/robot_base/gimbal_cmd`、`gimbal_state`：自有云台命令和状态。
- `/<robot>/cmd_shoot`：`example_interfaces/msg/UInt8`，数值为发射数量。
- `/<robot>/robot_base/shoot_cmd`：可同时指定数量和弹丸速度。
- `/<robot>/robot_base/set_light_color`：灯条颜色服务，0–4 对应关闭、红、蓝、黄、白。
- `/<robot>/chassis_odometry_gt`、`joint_states`、`chassis_imu`、`gimbal_imu`：状态反馈。
- `/<robot>/front_industrial_camera/image`、`camera_info`：工业相机。
- 哨兵额外提供 `/<robot>/livox/lidar` 和 `/<robot>/livox/imu`。
- TF 分别发布在 `/<robot>/tf` 和 `/<robot>/tf_static`，避免多机器人串扰。

交互式测试工具示例：

```bash
ros2 run recruitment_sim_robot_base test_chassis_cmd.py --ros-args -r __ns:=/infantry_robot/robot_base
ros2 run recruitment_sim_robot_base test_gimbal_cmd.py --ros-args -r __ns:=/infantry_robot/robot_base
ros2 run recruitment_sim_robot_base test_shoot_cmd.py --ros-args -r __ns:=/infantry_robot/robot_base
ros2 run recruitment_sim_robot_base test_light_color.py yellow --ros-args -r __ns:=/infantry_robot/robot_base
```

## 裁判系统边界

当前代码不启动或依赖裁判系统，也不发布 `/referee_system/*`。发射插件仍负责生成弹丸、
检测任意物理碰撞以及碰撞/超时后的弹丸回收；“是否击中有效装甲、扣血和比赛状态”留给
后续裁判系统重构实现。

第三方代码与模型来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
