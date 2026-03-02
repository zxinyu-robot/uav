# Diff-planner 整体架构

本文档描述 Diff-planner 工作空间的**代码与算法架构**，与 [nv_launch_structure.md](./nv_launch_structure.md)（launch 链与部署）互补。

---

## 一、总体分层与数据流

Diff-planner 按功能分为：**感知与地图** → **规划核心** → **轨迹执行与用户指令**，并支持**仿真**与**真机**两套数据源。

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                     用户 / 任务层                             │
                    │  multipoint (多点任务) │ RViz 2D Nav Goal │ 手动/预设航点      │
                    │  发布 /goal (PoseStamped) 或 /traj_start_trigger              │
                    └─────────────────────────────────────────────────────────────┘
                                                │
                                                ▼
┌───────────────────────────────────────────────────────────────────────────────────────────┐
│                           规划核心 (包名: diff_planner)                                    │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐ │
│  │ DiffReplanFSM   │───▶│ PlannerManager  │───▶│ PolyTrajOptimizer│───▶│ AStar           │ │
│  │ (状态机+触发)   │    │ (规划流程编排)   │    │ (MinJerk+LBFGS)  │    │ (path_searching)│ │
│  └────────┬────────┘    └────────┬────────┘    └────────┬────────┘    └────────┬────────┘ │
│           │                      │                       │                       │         │
│           │                      │                       └───────────┬───────────┘         │
│           │                      │                                   │                     │
│           │                      ▼                                   ▼                     │
│           │              ┌─────────────────┐              ┌─────────────────┐              │
│           │              │ GridMap          │              │ plan_env        │              │
│           │              │ (栅格/占据/膨胀)  │              │ (traj_opt 依赖)  │              │
│           │              │ plan_env         │              └─────────────────┘              │
│           │              └────────┬────────┘                                                │
│           │                       │                                                         │
│           │  订阅 odom、goal；发布 planning/trajectory、data_display、heartbeat              │
│           └──────────────────────┼─────────────────────────────────────────────────────────┤
│                                  │                                                         │
│  traj_server: 订阅 planning/trajectory → 发布 position_cmd (给控制/仿真)                    │
└──────────────────────────────────┼─────────────────────────────────────────────────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        ▼                           ▼                           ▼
┌───────────────┐         ┌───────────────────┐         ┌───────────────┐
│ 真机: 点云+里程计 │         │ GridMap 输入        │         │ 仿真: 虚拟点云  │
│ cloud + odom   │         │ grid_map/cloud     │         │ + pos_cmd→odom│
│ (LIO/EKF 等)   │         │ grid_map/odom      │         │ (run_in_sim)  │
└───────────────┘         └───────────────────┘         └───────────────┘
```

---

## 二、目录与包的分层对应

| 层级 | 目录 (src 下) | 包名 | 职责 |
|------|----------------|------|------|
| **规划入口与状态机** | diff_planner/plan_manage | diff_planner | 主节点 diff_planner_node（FSM）、traj_server、launch |
| **规划管理器** | diff_planner/plan_manage | diff_planner | PlannerManager：编排 init → 优化 → 写轨迹 |
| **环境表示** | diff_planner/plan_env | plan_env | GridMap：点云/深度→栅格、膨胀、虚拟墙 |
| **路径搜索** | diff_planner/path_searching | path_searching | AStar：栅格空间路径，供轨迹初值 |
| **轨迹优化** | diff_planner/traj_opt | traj_opt | PolyTrajOptimizer：MinJerk + 碰撞/动力学约束，LBFGS |
| **轨迹与可视化** | diff_planner/traj_utils | traj_utils | 轨迹容器、PolyTraj/MINCOTraj 消息、PlanningVisualization |
| **用户任务** | user_command/multipoint | multipoint | 多点任务：YAML 航点 → 按距离发布 /goal |
| **多机** | diff_planner/swarm_bridge | swarm_bridge | 轨迹/心跳转发，多机协同 |
| **真机感知/控制** | realflight_modules/* | faster_lio, ekf, px4ctrl, livox_ros_driver2 | LIO、EKF、飞控、雷达驱动 |
| **仿真** | uav_simulator/* | map_generator, mockamap, local_sensing_node, poscmd_2_odom, so3_control 等 | 地图、虚拟点云、指令转 odom |

---

## 三、核心算法与调用链

### 3.1 规划主流程（单次 Replan）

1. **DiffReplanFSM**（diff_replan_fsm.cpp）  
   - 根据状态（WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ → REPLAN_TRAJ 等）决定何时触发规划。  
   - 从 **odom** 取当前位姿/速度，从 **/goal** 或预设航点取目标。  
   - 调用 `planner_manager_->getLocalTarget()` 得到局部终点，再调用 `planner_manager_->reboundReplan()`。

2. **DiffPlannerManager**（planner_manager.cpp）  
   - `reboundReplan()`：  
     - **STEP 1**：`computeInitState()` 生成多项式初值（MinJerk 段、内点、时间分配）。  
     - **STEP 2**：`PolyTrajOptimizer::optimizeTrajectory()`：LBFGS 优化（碰撞、动力学、多机间距等）。  
     - **STEP 3**：`setLocalTrajFromOpt()` 写入轨迹容器并发布。  
   - 持有 `GridMap::Ptr grid_map_`、`PolyTrajOptimizer::Ptr ploy_traj_opt_`、`TrajContainer traj_`。

3. **PolyTrajOptimizer**（traj_opt，poly_traj_optimizer.cpp）  
   - 依赖 **plan_env（GridMap）** 做碰撞检测与约束点。  
   - 内部使用 **path_searching 的 AStar** 做初值或安全路径。  
   - 优化变量：内点 + 时间，目标为 MinJerk + 障碍/动力学/多机代价。

4. **GridMap**（plan_env，grid_map.cpp）  
   - 订阅 `grid_map/cloud`（点云）和 `grid_map/odom`（位姿），或深度图+相机位姿。  
   - 维护局部栅格、膨胀、虚拟天花板/地面。  
   - 为轨迹优化提供占据/距离信息。

5. **AStar**（path_searching，dyn_a_star.cpp）  
   - 基于 GridMap 的栅格做 3D A* 搜索。  
   - 被 PolyTrajOptimizer 调用于生成无碰路径或初值。

**总结**：FSM 驱动 → PlannerManager 编排 → PolyTrajOptimizer 优化（用 GridMap + AStar）→ 轨迹写入容器并经 traj_server 发出。

### 3.2 轨迹执行

- **traj_server**（plan_manage）：  
  - 订阅 `planning/trajectory`（PolyTraj）、`planning/yaw`、`heartbeat`。  
  - 按时间采样轨迹，发布 **quadrotor_msgs::PositionCommand** 到 `position_cmd`（launch 中常 remap 为 `/setpoints_cmd` 等给 px4ctrl 或仿真）。

---

## 四、主要话题与节点关系

（以下为包内默认或 launch 中常见 remap 后的逻辑关系。）

| 话题/接口 | 类型 | 发布方 | 订阅方 | 说明 |
|-----------|------|--------|--------|------|
| **odom_world** | nav_msgs::Odometry | EKF/仿真 | diff_planner_node | 当前位姿与速度 |
| **/goal** | geometry_msgs::PoseStamped | multipoint / RViz | diff_planner_node | 目标点 |
| **grid_map/cloud** | sensor_msgs::PointCloud2 | LIO/仿真 pcl_render | GridMap (plan_env) | 点云 |
| **grid_map/odom** | nav_msgs::Odometry | 同上 / 仿真 | GridMap | 建图用位姿 |
| **planning/trajectory** | traj_utils::PolyTraj | diff_planner_node | traj_server | 规划结果轨迹 |
| **planning/yaw** | (yaw 指令) | multipoint 等 | traj_server | 航向 |
| **planning/data_display** | traj_utils::DataDisp | diff_planner_node | 可视化 | 调试显示 |
| **planning/heartbeat** | std_msgs::Empty | traj_server | FSM / 多机 | 心跳 |
| **position_cmd** (/setpoints_cmd) | quadrotor_msgs::PositionCommand | traj_server | px4ctrl / 仿真 | 位置控制指令 |
| **planning/broadcast_traj_*** | traj_utils::MINCOTraj | 本机 planner | swarm_bridge→他机 | 多机轨迹同步 |

multipoint 还订阅 `odom_topic`（参数）、发布 `/goal`；可选发布 yaw、起降指令等，与规划器通过 **/goal** 解耦。

---

## 五、仿真与真机两套数据流

### 5.1 真机（Exp LIO）

- **感知**：LIO（faster_lio）→ 点云；EKF（ekf）→ `/ekf/ekf_odom`。  
- **规划**：launch 将 `grid_map/cloud` remap 到 `/laserMapping/cloud_registered`，`odom_world` 到 `/ekf/ekf_odom`。  
- **控制**：traj_server 发布 position_cmd → px4ctrl → 飞控。  
- **任务**：multipoint 读取 YAML 航点，按距离发布 `/goal`。

### 5.2 仿真（Sim）

- **地图**：map_generator 或 mockamap 发布全局点云。  
- **感知**：local_sensing_node（pcl_render_node）根据仿真 odom 和全局图生成局部点云，发给 GridMap。  
- **位姿**：poscmd_2_odom 将 planning/pos_cmd 积分为 odom，供规划与感知使用。  
- **规划**：同一套 diff_planner_node + GridMap + traj_server，仅输入源改为仿真 topic。  
- run_sim_single.launch / run_sim_swarm.launch 通过 run_in_sim.xml、simulator.xml 串起上述节点（见 nv_launch_structure.md）。

---

## 六、多机与 swarm_bridge

- 每机有独立的 diff_planner_node、traj_server（带 drone_id 命名空间或前缀）。  
- **swarm_bridge**：订阅本机 `planning/broadcast_traj_send`，转发到他机 `planning/broadcast_traj_recv`；PlannerManager 可订阅他机轨迹做避障（PolyTrajOptimizer 内 swarm 相关代价）。  
- run_sim_swarm.launch 先起 bridge，再为每机 include run_in_sim.xml。

---

## 七、与 Launch 的对应关系

- **入口**：真机用 `run_exp_single_lio.launch`（diff_planner）+ multipoint + px4ctrl + EKF + LIO；仿真用 `run_sim_single.launch` 或 `run_sim_swarm.launch`。  
- **包名**：代码与 launch 中统一使用 **diff_planner**、**multipoint**、**plan_env**、**path_searching**、**traj_opt**、**traj_utils**；与目录名 plan_manage、user_command 等不一致处见 nv_launch_structure.md。

以上即为 Diff-planner 的**整体架构**：分层清晰，规划核心为 FSM → PlannerManager → PolyTrajOptimizer + GridMap + AStar，轨迹由 traj_server 输出，用户与任务通过 /goal 和 multipoint 接入。
