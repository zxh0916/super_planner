# SUPER Core Refactor Plan

## 目标

将 SUPER 的核心规划能力从 ROS1/ROS2 节点中剥离，构建成可动态链接的 C++ 核心库，并通过 pybind11 暴露为 Python 可 `import` 的模块。目标不是把在线重规划压成一个无状态函数，而是暴露一个有状态 `PlannerSession`，用多个 callable 驱动地图、状态、目标、重规划和轨迹采样，从而保留现有完整重规划能力。

核心能力范围：

- 输入静态 PCD 地图和增量局部点云。
- 输入无人机本体状态。
- 输入目标点和可选 yaw。
- 维护 ROG-Map、上一条 committed trajectory、backup trajectory、重规划状态和时间。
- 输出多项式轨迹和当前时刻控制指令。

不在第一阶段处理：

- ROS topic、timer、message、RViz 可视化。
- TF 坐标变换。Python 调用方必须提供 world frame 点云和状态。
- 电机级控制。输出仍是现有 flatness setpoint 级别的 `PositionCommand`。

## 现有代码耦合点

当前核心链路分布如下：

- `rog_map::ROGMap` 是非 ROS 地图核心，但它是抽象类，要求子类实现 `getSystemWalltimeNow()`。
- `rog_map::ROGMapROS` 负责 ROS odom/cloud callback、timer、可视化发布，并继承 `ROGMap`。
- `super_planner::SuperPlanner` 包含 A*、安全走廊、探索轨迹优化、backup 轨迹优化、yaw 轨迹优化和 committed trajectory 管理。
- `SuperPlanner` 当前依赖 `rog_map::ROGMapROS::Ptr`，但实际大部分调用只需要 `ROGMap` 的地图查询和更新 API。
- `SuperPlanner` 还依赖 `ros_interface::RosInterface::Ptr`，其中核心只需要日志、时间和可视化钩子；ROS1/ROS2 版本主要用于发布 marker、点云和获取 ROS time。
- `fsm::Fsm` / `FsmRos1` / `FsmRos2` 负责目标接收、状态机、timer、轨迹采样和命令发布。要做 Python 库时，应把状态机逻辑迁移到 ROS 无关的 session，而不是继续绑定 ROS FSM。

关键结论：需要新增 ROS-free adapter，而不是重写规划算法。

## 推荐 callable 数量

保留完整重规划能力时，推荐暴露 **7 个核心 callable**，加 **2 个辅助 callable**。

### 1. `PlannerSession(config_path: str)`

职责：

- 加载现有 YAML 配置。
- 初始化非 ROS 时间/日志/可视化接口。
- 初始化非 ROS ROG-Map。
- 初始化 `SuperPlanner`。
- 初始化状态机状态：`INIT`、`WAIT_GOAL`、`GENERATE_TRAJ`、`FOLLOW_TRAJ`、`EMER_STOP`。
- 保存 `last_replan_time`、`started`、`finish_plan`、`goal_info` 等 session 状态。

建议 Python 用法：

```python
import super_planner_py as super

planner = super.PlannerSession("super_planner/config/static_high_speed.yaml")
```

内部不启动线程，不创建 timer。Python 调用方通过 `step(time_s)` 显式推进。

### 2. `load_static_pcd(pcd_path: str, clear: bool = True) -> MapStats`

职责：

- 读取 PCD 文件。
- 可选清空已有地图。
- 将 PCD 作为静态占据点写入 ROG-Map。
- 更新膨胀地图，必要时更新 ESDF。

输入：

- `pcd_path`: 绝对路径或相对路径。
- `clear`: 是否先清空地图。

输出 `MapStats`：

- `success`
- `point_count`
- `map_empty`
- `message`

注意：

- 当前 `ROGMap::init()` 已支持 `load_pcd_en`，但它从 YAML 读 `pcd_name`。新 API 应直接接受路径，避免 Python 侧为每张地图改 YAML。
- PCD 作为静态地图时，不需要模拟 LiDAR 渲染；如果要复刻现有仿真感知链路，可由 Python 侧继续传局部点云到 `update_sensing()`。

### 3. `update_sensing(points, state, time_s: float) -> MapUpdateResult`

职责：

- 输入一帧局部点云和本体状态。
- 将状态写入地图和 planner。
- 调用 ROG-Map 的概率地图更新。
- 保留在线重规划所需的最新障碍信息。

输入：

- `points`: numpy 数组，形状 `N x 3` 或 `N x 4`。
  - `N x 3`: `x, y, z`，intensity 默认 1。
  - `N x 4`: `x, y, z, intensity`。
- `state`: Python `RobotState` 或 dict。
  - `position: [x, y, z]`
  - `velocity: [vx, vy, vz]`
  - `acceleration: [ax, ay, az]`
  - `jerk: [jx, jy, jz]`
  - `quat: [w, x, y, z]`
  - `yaw`
- `time_s`: 调用方提供的单调时间或仿真时间。

输出 `MapUpdateResult`：

- `success`
- `input_point_count`
- `used_point_count`
- `robot_state_received`
- `message`

约束：

- 输入点云默认已经在 world frame。
- 如果 `points` 为空，只更新本体状态，不更新地图。
- 如果当前配置 `rog_map/ros_callback/enable` 为 true，应在 core session 初始化时强制覆盖为 false，避免 `ROGMap::updateMap()` 拒绝 API 更新。

### 4. `set_goal(position, yaw: Optional[float] = None) -> GoalResult`

职责：

- 设置目标点和可选 yaw。
- 调用地图最近非占据点检查，将深度占据目标拒绝或投影到附近安全点。
- 设置 `new_goal = true`，使下一次 `step()` 触发 `PlanFromRest` 或重规划。

输入：

- `position: [x, y, z]`
- `yaw: None` 表示自动 yaw。

输出 `GoalResult`：

- `accepted`
- `goal_position`
- `goal_yaw`
- `message`

行为对应当前 `Fsm::setGoalPosiAndYaw()`，但去掉 ROS message 输入。

### 5. `step(time_s: float) -> StepResult`

职责：

- 设置内部模拟时间。
- 执行一次状态机推进。
- 在无轨迹时调用 `PlanFromRest()`。
- 在 `FOLLOW_TRAJ` 状态下按配置 `replan_rate` 调用 `ReplanOnce()`。
- 处理返回码和状态迁移。

输入：

- `time_s`: 当前时间。

输出 `StepResult`：

- `success`
- `ret_code`
- `state`
- `new_trajectory`
- `trajectory_finished`
- `used_backup`
- `message`

状态机建议：

- `INIT`: 等待有效 robot state 和 goal。
- `WAIT_GOAL`: 有新 goal 时进入 `GENERATE_TRAJ`。
- `GENERATE_TRAJ`: 调 `PlanFromRest()`，成功后进入 `FOLLOW_TRAJ`。
- `FOLLOW_TRAJ`: 到重规划周期时调 `ReplanOnce()`；轨迹结束后根据是否到达目标回到 `WAIT_GOAL` 或 `GENERATE_TRAJ`。
- `EMER_STOP`: 输出 backup 轨迹段，下一步尝试回到 `WAIT_GOAL` 或重新规划。

### 6. `get_trajectory() -> TrajectoryBundle`

职责：

- 导出当前 committed position/yaw trajectory。
- 导出 backup 起始时间和是否处于 backup。
- 导出最近一次重规划诊断。

输出 `TrajectoryBundle`：

- `position_durations: np.ndarray[piece]`
- `position_coeffs: np.ndarray[piece, 3, 8]`
- `yaw_durations: np.ndarray[piece]`
- `yaw_coeffs: np.ndarray[piece, 1, 8]`
- `start_time`
- `total_duration`
- `has_backup`
- `backup_start_time`
- `ret_code`

注意：

- 当前 `Trajectory` 内部由 `Piece` 组成，每段有 duration 和 coefficient matrix。
- Python 侧应拿到纯 numpy 数据，不依赖 ROS message。

### 7. `sample_command(time_s: float) -> PositionCommand`

职责：

- 从 committed trajectory 在 `time_s` 对应轨迹时刻采样。
- 输出位置、速度、加速度、jerk、yaw、yaw_rate。
- 调用 `geometry_utils::convertFlatOutputToAttAndOmg()` 输出姿态、角速度和推力。

输出 `PositionCommand`：

- `position`
- `velocity`
- `acceleration`
- `jerk`
- `yaw`
- `yaw_rate`
- `attitude_rpy`
- `angular_velocity`
- `thrust`
- `trajectory_flag`
- `trajectory_finished`

语义对应现有 `/planning/pos_cmd`，但不依赖 `quadrotor_msgs` 或 `mars_quadrotor_msgs`。

### 辅助 1. `reset(clear_map: bool = False) -> None`

职责：

- 清空 goal、轨迹、FSM 状态、重规划日志。
- 可选清空地图。

用于批量实验或重新开始 episode。

### 辅助 2. `get_debug_state() -> Diagnostics`

职责：

- 返回内部状态，方便 Python 调试和测试。

输出建议：

- `fsm_state`
- `has_goal`
- `has_map`
- `has_trajectory`
- `robot_state`
- `last_ret_code`
- `last_module_time`
- `guide_path`
- `safe_corridors`
- `latest_message`

## 新增核心类型

### `CoreROGMap`

位置建议：

- `rog_map/include/rog_map/core_rog_map.h`
- `rog_map/src/rog_map/core_rog_map.cpp`

职责：

- 继承 `rog_map::ROGMap`。
- 实现 `getSystemWalltimeNow()`，返回 session 当前时间。
- 暴露非 ROS 的状态和点云更新 API。
- 不包含 subscriber、publisher、timer、TF 或 marker。

核心方法：

```cpp
class CoreROGMap : public rog_map::ROGMap {
public:
    explicit CoreROGMap(const std::string& cfg_path);
    void setTime(double time_s);
    void setRobotState(const super_utils::RobotState& state);
    MapStats loadStaticPcd(const std::string& pcd_path, bool clear);
    MapUpdateResult updateSensingCloud(const rog_map::PointCloud& cloud,
                                       const super_utils::RobotState& state,
                                       double time_s);
private:
    double current_time_{0.0};
    const double getSystemWalltimeNow() override;
};
```

### `NoRosInterface`

位置建议：

- `super_planner/include/ros_interface/no_ros_interface.hpp`

职责：

- 实现 `ros_interface::RosInterface`。
- 提供 `setSimTime()` / `getSimTime()`。
- 日志写 stdout 或可注入 callback。
- 所有可视化方法默认 no-op，但保存必要 debug 数据可选。

这样 `SuperPlanner` 不需要直接知道 Python 或 ROS。

### `PlannerSession`

位置建议：

- `super_planner/include/super_core/planner_session.h`
- `super_planner/src/super_core/planner_session.cpp`

职责：

- 聚合 `CoreROGMap`、`NoRosInterface`、`SuperPlanner`。
- 承接现有 `Fsm` 的非 ROS 状态机。
- 提供 pybind11 直接绑定的 C++ API。

注意：

- `SuperPlanner` 目前构造函数接受 `rog_map::ROGMapROS::Ptr`。应改成接受 `std::shared_ptr<rog_map::ROGMap>` 或一个更窄的地图接口。为了最小改动，优先把类型从 `ROGMapROS::Ptr` 替换为 `std::shared_ptr<rog_map::ROGMap>`，因为 `Astar`、`CorridorGenerator` 也主要依赖地图查询 API。
- ROS1/ROS2 wrapper 可以继续传入 `ROGMapROS`，因为它继承 `ROGMap`。

## pybind11 模块

模块名建议：

```python
import super_planner_py
```

绑定位置建议：

- `super_planner/python/bindings.cpp`

绑定内容：

- `PlannerSession`
- `RobotState`
- `MapStats`
- `MapUpdateResult`
- `GoalResult`
- `StepResult`
- `TrajectoryBundle`
- `PositionCommand`
- enum: `RetCode`, `PlannerState`

numpy 转换规则：

- 点云输入要求 `float64` 或 `float32` contiguous array。
- 内部转换为 `pcl::PointCloud<pcl::PointXYZI>`。
- 轨迹系数输出为 numpy array，避免 Python 侧解析 C++ 对象。

## Python 调用时序

典型静态 PCD + 在线重规划流程：

```python
import super_planner_py as super

planner = super.PlannerSession("super_planner/config/static_high_speed.yaml")
planner.load_static_pcd("/path/to/map.pcd")

planner.update_sensing(local_cloud_np, robot_state, time_s=0.0)
planner.set_goal([20.0, 0.0, 1.5])

for k in range(1000):
    t = k * 0.01
    planner.update_sensing(local_cloud_np, robot_state, t)
    step = planner.step(t)
    cmd = planner.sample_command(t)
    send_to_controller(cmd)
```

如果只做全局静态地图规划，可以先 `load_static_pcd()`，之后每个周期只更新 `state`，局部点云可以为空。

## 构建方案

新增 CMake 目标：

- `super_core`: 不依赖 ROS，只链接 Eigen、PCL、yaml、fmt、现有优化代码和 ROG-Map core。
- `super_planner_py`: pybind11 module，链接 `super_core`。
- 现有 ROS targets 继续保留，链接 `super_core` 或继续按原方式构建，第一阶段以不破坏 ROS demo 为优先。

需要拆分的依赖：

- `rog_map` core 和 `rog_map_ros` 分离。
- `ros_interface/ros1`、`ros_interface/ros2` 保留为 ROS wrapper。
- `NoRosInterface` 放在 core 可编译区域，不能 include ROS header。

## 迁移步骤

1. 新增 `NoRosInterface`，确保 `SuperPlanner` 可在无 ROS node 下获得时间和日志。
2. 新增 `CoreROGMap`，确保可通过 API 更新状态和点云。
3. 将 `SuperPlanner`、`Astar`、`CorridorGenerator` 中的 `ROGMapROS::Ptr` 改为 `std::shared_ptr<ROGMap>`。
4. 新增 `PlannerSession`，复制并压缩 `Fsm` 中的状态机逻辑，去掉 ROS timer/topic/message。
5. 新增纯 C++ smoke test，验证 PCD、状态、目标到轨迹生成链路。
6. 新增 pybind11 module 和 Python smoke test。
7. 回归 ROS1/ROS2 构建，确认原 launch 文件仍可运行。

## 测试计划

### C++ core tests

- 构造 `PlannerSession`。
- 加载 `mars_uav_sim/perfect_drone_sim/pcd/random_map_24_6635.pcd`。
- 设置初始状态 `(0, -50, 1.5)`。
- 设置目标。
- 调用 `step()`，验证返回成功或明确失败码。
- 验证 `get_trajectory()` 的 piece 数、duration、系数非空。
- 调用 `sample_command()`，验证 position、velocity、acceleration、yaw 均为有限数。

### Python tests

- `import super_planner_py`。
- numpy 点云输入 `N x 3` 和 `N x 4` 都能工作。
- 空点云只更新状态，不崩溃。
- 连续调用 `step()` 能维持状态，不重复从零规划。
- 加入新障碍点云后，下一轮重规划可返回新轨迹或明确失败码。

### ROS regression

- ROS1: `catkin_make -DBUILD_TYPE=Release`。
- ROS1 demo: `roslaunch mission_planner benchmark_high_speed.launch`。
- ROS2: `colcon build --symlink-install`。
- 确认 `/planning/pos_cmd` 和 `/planning_cmd/poly_traj` 语义不变。

## 风险与注意事项

- 完整重规划能力依赖历史轨迹和时间，不能设计成真正 stateless function。
- 当前 `ROGMap::updateMap()` 在 `ros_callback_en=true` 时会拒绝 API 更新；core 初始化必须强制禁用 ROS callback 或要求配置文件设置为 false。
- 当前日志路径依赖 `ROOT_DIR` 和 `DEBUG_FILE_DIR`，Python wheel/so 环境下需要允许关闭文件日志或指定 log dir。
- `SuperPlanner` 的可视化调用很多，`NoRosInterface` 必须完整实现 no-op，否则会阻塞编译。
- 轨迹系数阶数当前按 7 阶、多项式系数 8 列处理；Python API v1 可以固定该格式，不做可变阶数泛化。
- 若要把现有 ROS wrapper 迁移到 `PlannerSession`，应作为第二阶段，避免第一阶段同时改变 ROS 行为。

## 最小可交付版本

第一版建议交付：

- `PlannerSession`
- `load_static_pcd`
- `update_sensing`
- `set_goal`
- `step`
- `get_trajectory`
- `sample_command`
- Python import smoke test

暂缓：

- debug 可视化数据完整导出。
- ROS wrapper 重写为 session adapter。
- 打包 wheel。
- 多线程安全优化。
