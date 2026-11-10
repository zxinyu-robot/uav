# UAV Autonomous Navigation

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

> 受限环境下的无人飞行器自主导航案例，聚焦无 GPS、弱网 / 断网和工业巡检场景。

## 项目状态

| 项目 | 状态 |
| --- | --- |
| 维护状态 | Active |
| 项目类型 | 公开案例与演示素材 |
| 验证范围 | 脱敏模拟器与公开工程说明 |
| 不包含 | 历史商业系统、完整系统实现和厂商代码 |

## 项目简介

本仓库承载案例 A 的可公开演示素材与工程说明，用于说明受限环境下无人机自主导航的系统边界和工程方法。内容不代表完整历史系统或军用飞行器代码。

## 演示

完整说明见 [`docs/demo-case-a.md`](docs/demo-case-a.md)。

| 演示 | 内容 |
| --- | --- |
| [军工飞行器导航模块](assets/demo/uav_gps_denied_15s.mp4) | 脱敏模拟器展示 |
| [黑暗环境自主巡检](assets/demo/uav_dark_patrol.mp4) | 无 GPS 无人机简支桥梁自主巡检流程 |

## 工程范围

- 受限环境中的无人机与多机导航。
- 几何 3D SLAM 与传统运动控制。
- 基于消息中间件的任务执行、状态机调度与系统集成。
- 嵌入式部署和通信受限条件下的任务运行。

## 限制与说明

历史项目的商业细节、完整系统和厂商实现不在本仓库中。演示素材用于说明案例背景与边界，不构成军用或生产系统性能声明。

## 关联项目

- [ForkSpatial-Runtime-Engine](https://github.com/zxinyu-robot/ForkSpatial-Runtime-Engine)：空间状态和运行时底座。
- [quadruped-crossfloor-nav-ws](https://github.com/zxinyu-robot/quadruped-crossfloor-nav-ws)：案例 B，四足机器人跨楼层导航。

## 许可证

[MIT](LICENSE)
