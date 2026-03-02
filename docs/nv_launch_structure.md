# /home/nv 目录与 Launch 链梳理

## 一、/home/nv 顶层结构概览

| 路径 | 类型 | 说明 |
|------|------|------|
| **Diff-planner** | ROS 工作空间 (catkin) | 主项目：规划 + 仿真 + 真机模块，含完整 launch |
| **faster-lio** | ROS 工作空间 | 独立 Faster-LIO 仓库，与 Diff-planner 内 realflight_modules/faster-lio 可二选一 |
| **PX4_Param_Modify** | 非 ROS | PX4 参数脚本 + 文档，无 launch |
| **uav_ws** | 空/占位 | 仅 .git，无 src、无 launch |
| **Lib** | 依赖库 | livox_ros_driver_ws、nlopt、librealsense、cv_bridge_pkgs 等 |
| **Depend** | 依赖/脚本 | 其他依赖或部署用资源 |

**结论**：与“真机实验 / launch 链”直接相关的主要是 **Diff-planner**；**faster-lio** 为独立 LIO 工程，可与 Diff-planner 叠用或替代其内嵌 faster-lio。

---

## 二、Diff-planner 工作空间结构

### 2.1 包与目录对应（包名 ≠ 目录名时已标出）

| 目录路径 | 包名 (package.xml \<name\>) | 说明 |
|----------|----------------------------|------|
| src/diff_planner/**plan_manage** | **diff_planner** | 规划核心 + 实验/仿真 launch 入口 |
| src/diff_planner/plan_env | plan_env | 环境/栅格 |
| src/diff_planner/path_searching | path_searching | 路径搜索 |
| src/diff_planner/traj_opt | traj_opt | 轨迹优化 |
| src/diff_planner/traj_utils | traj_utils | 轨迹工具 |
| src/diff_planner/swarm_bridge | swarm_bridge | 多机通信 |
| src/diff_planner/drone_detect | drone_detect | 无人机检测 |
| src/user_command/multipoint | **multipoint** | 多点任务（包名 multipoint，非 usercommand） |
| src/realflight_modules/faster-lio | **faster_lio** | LIO 建图（包名带下划线） |
| src/realflight_modules/ekf_pose | **ekf** | 融合位姿（包名 ekf，非 ekf_pose） |
| src/realflight_modules/px4ctrl | px4ctrl | 飞控控制 |
| src/realflight_modules/livox_ros_driver2 | livox_ros_driver2 | Livox 驱动 |
| src/Utils/manual_take_over | manual_take_over | 手动接管 |
| src/Utils/odom_visualization | odom_visualization | 里程计可视化 |
| src/uav_simulator/fake_drone | poscmd_2_odom | 仿真：指令转里程计 |
| src/uav_simulator/local_sensing | local_sensing_node | 仿真：局部感知 |
| src/uav_simulator/so3_control | so3_control | 仿真：控制 |
| src/uav_simulator/so3_quadrotor_simulator | so3_quadrotor_simulator | 仿真：机体 |
| src/uav_simulator/map_generator | map_generator | 仿真：地图生成 |
| src/uav_simulator/mockamap | mockamap | 仿真：地图 |
| 其他 | quadrotor_msgs, realsense2_camera, vins 等 | 消息/相机/VIO |

**易错点**：  
- 找“规划”相关 launch 时要用包名 **diff_planner**，不要用 **plan_manage**。  
- 找“多点”要用 **multipoint**，不要用 **usercommand**。  
- 找 EKF 要用 **ekf**，不要用 **ekf_pose**。  
- LIO 包名是 **faster_lio**（下划线），目录是 faster-lio（中划线）。

---

## 三、Launch 链梳理

### 3.1 实验真机 LIO 链路（Exp LIO）

真机实验时，通常按“感知 → 定位 → 控制 → 规划 → 任务”顺序启动，**彼此无单一顶层 launch 串联**，需分别启动或由外部脚本/部署工具按顺序调：

```
┌─────────────────────────────────────────────────────────────────────────┐
│  1. 激光 / 驱动（二选一或组合）                                            │
│     • livox_ros_driver2（如 MID360）                                       │
│     • 或其它雷达驱动                                                        │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  2. LIO 建图（包名: faster_lio）                                           │
│     • realflight_modules/faster-lio/launch/mapping_mid360.launch          │
│     • 依赖: $(find faster_lio), $(find livox_ros_driver2)                  │
│     输出: /laserMapping/cloud_registered 等                                │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  3. 位姿融合（包名: ekf）                                                  │
│     • realflight_modules/ekf_pose/launch/ekf_lidar.launch                 │
│     • 依赖: $(find ekf)   ← 注意包名是 ekf 不是 ekf_pose                    │
│     输出: /ekf/ekf_odom                                                   │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  4. 规划（包名: diff_planner）                                            │
│     • diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch        │
│     • include: advanced_param_exp.xml（diff_planner_node + 参数）          │
│     • 同 launch 内启动 traj_server（包 diff_planner）                      │
│     • 依赖: cloud_topic=/laserMapping/cloud_registered, odom=/ekf/ekf_odom  │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  5. 控制（包名: px4ctrl）                                                 │
│     • realflight_modules/px4ctrl/launch/run_ctrl_lio.launch                │
│     • 依赖: $(find px4ctrl)                                               │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  6. 多点任务（包名: multipoint）                                           │
│     • user_command/multipoint/launch/multipointplan_exp_lio.launch         │
│     • 依赖: $(find multipoint)  ← 包名 multipoint 不是 usercommand          │
└─────────────────────────────────────────────────────────────────────────┘
                                    ↓
┌─────────────────────────────────────────────────────────────────────────┐
│  7. 可视化（可选）                                                        │
│     • diff_planner/plan_manage/launch/exp/exp_rviz.launch                  │
│     • $(find diff_planner)/launch/include/exp.rviz                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**若部署脚本里用错了包名**，会出现你之前遇到的错误，例如：  
- `run_exp_single_lio.launch` 在包 **plan_manage** 里 → 应改为包 **diff_planner**  
- `multipointplan_exp_lio.launch` 在包 **usercommand** 里 → 应改为包 **multipoint**  
- `ekf_lidar.launch` 在包 **ekf_pose** 里 → 应改为包 **ekf**  
- `mapping_mid360.launch` 在包 **faster-lio** 里 → 应改为包 **faster_lio**（且需保证该包来自当前 source 的工作空间）

---

### 3.2 仿真链路（Sim）

仿真由 **run_sim_single.launch** 或 **run_sim_swarm.launch** 作为入口，内部通过 include 形成一条链：

```
run_sim_single.launch (diff_planner)
├── map_generator (random_forest)
├── run_in_sim.xml
│   ├── advanced_param_sim.xml   → diff_planner_node + 仿真参数
│   ├── traj_server (diff_planner)
│   ├── simulator.xml
│   │   ├── poscmd_2_odom (fake_drone)
│   │   ├── odom_visualization
│   │   ├── local_sensing_node (pcl_render_node)
│   │   └── (可选) so3_control, so3_quadrotor_simulator
│   └── manual_take_over/take_over_drone.launch
├── multipoint/multipointplan_sim.launch
└── rviz (sim.rviz)
```

**run_sim_swarm.launch** 在以上基础上先 include **swarm_bridge**，再为多机多次 include **run_in_sim.xml**，最后共用一个 rviz。

**依赖包汇总（仿真）**：  
diff_planner, swarm_bridge（仅 swarm）, multipoint, manual_take_over, so3_control, local_sensing_node, odom_visualization, poscmd_2_odom, map_generator。

---

### 3.3 各 launch 的 $(find xxx) 依赖汇总

| 被引用的包名 | 出现在哪些 launch/xml 中 |
|--------------|---------------------------|
| **diff_planner** | run_exp_single_lio, run_sim_*, advanced_param_*, run_in_sim.xml, exp_rviz |
| **multipoint** | multipointplan_exp_lio, multipointplan_sim |
| **swarm_bridge** | run_sim_swarm |
| **manual_take_over** | run_in_sim.xml |
| **so3_control** | simulator.xml（部分被注释） |
| **local_sensing_node** | simulator.xml |
| **faster_lio** | mapping_*.launch（Diff-planner 内 realflight_modules/faster-lio） |
| **livox_ros_driver2** | mapping_mid360.launch |
| **px4ctrl** | run_ctrl_lio, run_ctrl_vio |
| **ekf** | ekf_lidar.launch（包名必须是 ekf） |

---

## 四、faster-lio 与 Diff-planner 的关系

- **/home/nv/faster-lio**：独立仓库，包名 **faster_lio**，自带 launch（如 mapping_mid360.launch 等）。  
- **Diff-planner/src/realflight_modules/faster-lio**：同源或副本，包名同样为 **faster_lio**。

若同时存在两个工作空间并都 source，ROS 会解析到哪一个 **faster_lio** 取决于 **source 顺序**（后 source 的会覆盖前面的）。  
真机部署时要么：  
- 只使用 Diff-planner 工作空间（包含内嵌 faster-lio），或  
- 只使用 /home/nv/faster-lio 并配合 Diff-planner（不包含内嵌 faster-lio），  
并保证部署脚本里用 **faster_lio** 调用的 launch 来自当前生效的工作空间，且不要写 **faster-lio**（包名无中划线）。

---

## 五、PX4_Param_Modify、uav_ws

- **PX4_Param_Modify**：无 ROS 包、无 launch，仅脚本与文档，与 launch 链无直接关联。  
- **uav_ws**：当前无 src、无 launch，若未来作为“总入口”工作空间，需要在这里放顶层 launch 并正确依赖 Diff-planner / faster-lio 等包名（如上表）。

---

## 六、建议检查清单（真机部署时）

1. **工作空间**：确认只 source 一套包含规划 + LIO + EKF + px4ctrl 的工作空间（通常即 Diff-planner），避免多 workspace 混用导致找到错误包。  
2. **包名**：所有脚本/launch 中用到的名字改为：**diff_planner**、**multipoint**、**ekf**、**faster_lio**、**px4ctrl**，不要用 plan_manage、usercommand、ekf_pose、faster-lio。  
3. **启动顺序**：驱动 → LIO → EKF → 规划 run_exp_single_lio → px4ctrl → multipointplan_exp_lio →（可选）exp_rviz。  
4. **faster_lio 来源**：若从 /home/nv/faster-lio 单独拉 LIO，则不要再在 Diff-planner 里重复拉 realflight_modules/faster-lio，避免两个 faster_lio 冲突。

按上述关联和包名统一后，launch 链与 nv 目录内容可一一对应，部署报错会大幅减少。
