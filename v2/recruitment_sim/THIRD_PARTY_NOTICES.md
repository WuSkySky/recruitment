# Third-party notices

本工作空间基于以下开源项目中的部分代码和资源进行适配。原文件中的版权声明继续保留：

- `pb2025_robot_description`：机器人 XMacro、工业相机与 Mid360 资源；包元数据声明为 MIT。
- `rmoss_gazebo`：Robot Base、弹丸物理事件与 Gazebo 插件；Apache-2.0，部分插件文件单独声明 MIT。
- `rmoss_gz_resources`：底盘、装甲、灯条、测速模块和弹丸资源；Apache-2.0。
- `rmoss_interfaces`：控制消息定义；Apache-2.0。
- `sdformat_tools`：SDF→URDF 转换代码；Apache-2.0。
- `xmacro`：运行时 Python 依赖；MIT。
- `aiohttp`：选手端 HTTP/WebSocket 服务；Apache-2.0。
- `aiortc`：选手端 WebRTC 实现；BSD-3-Clause。
- `PyAV`：FFmpeg 的 Python 绑定，用于视频帧编码；BSD-3-Clause。
- `Vue.js`、`Vite`：选手端页面框架与构建工具；MIT。
- `ROS2_RM_Navigation` / `pb_rm_simulation`：RMUL 2026 3V3 场地 SDF 与网格；MIT，Copyright (c) 2026 MY_nav Contributors。

完整 Apache 2.0 文本保存在 `LICENSES/Apache-2.0.txt`，MIT 文本保存在
`LICENSES/MIT.txt`；场地上游的原始 MIT 声明另存为 `LICENSES/MY_nav-MIT.txt`。
RMOSS 参考源码位于相邻的 `v2/rmu_ws`；场地来源为
[`ROS2_RM_Navigation`](https://github.com/laohao78/ROS2_RM_Navigation/tree/master/src/gazebo/pb_rm_simulation/world/RMUL2026H_world)。
