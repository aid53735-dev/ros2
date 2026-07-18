# 架构设计(ARCHITECTURE)

## 1. 总体架构

```
                            ┌────────────────────────────────────┐
                            │      外部操作端(唯一控制入口)        │
                            │  /nav/mission_cmd (MissionCommand)  │
                            └──────────────┬─────────────────────┘
                                           ▼
 ┌──────────────┐  /nav/traj/cmd   ┌──────────────┐
 │ Trajectory    │◀────────────────│ Mission       │◀── /nav/fault ──────────┐
 │ Tracking      │                 │ Manager       │◀── /nav/loc_valid ──┐   │
 │ trajectory_   │── /nav/traj/ ──▶│ mission_node  │                     │   │
 │ node          │   reference     └──────┬───────┘                      │   │
 └──────┬───────┘        │               /nav/mission_state              │   │
        │ ▲              ▼                                               │   │
        │ │       ┌──────────────┐ /cmd_vel ┌──────────────┐             │   │
        │ │       │ Navigation    │────────▶│ tower_nav_sim │             │   │
        │ │       │ Controller    │         │ sim_node      │             │   │
        │ │       │ controller_   │         └──────┬───────┘             │   │
        │ │       │ node          │   /imu/data、/wheel/encoder          │   │
        │ │       └──────▲───────┘                 ▼                     │   │
        │ │ /nav/pose    │ /nav/fault      ┌──────────────┐──────────────┘   │
        │ └──────────────┼─────────────────│ Localization  │                 │
        │                │                 │ localization_ │── /nav/pose ──┐ │
        │                │                 │ node          │               │ │
        │                │                 └──────────────┘               │ │
        │                │                                                ▼ │
        │         ┌──────┴────────┐  心跳/偏差/急停/超时监测  ┌──────────────┐│
        │         │ Exception      │◀───────────────────────│ (各话题)      ││
        │         │ Manager        │── /nav/fault ──────────▶└──────────────┘│
        │         │ exception_node │── /cmd_vel=0(安全停车)                  │
        │         └───────────────┘                                         │
        │                                  ┌──────────────┐                 │
        └── /nav/traj/reference ──────────▶│ Status        │◀───────────────┘
                                           │ Manager       │──▶ /nav/status
                                           └──────────────┘
```

七个节点(六导航 + 一仿真),单一职责、话题/服务解耦:

| 节点 | 职责 |
| --- | --- |
| `localization_node` | 传感融合定位,发布 `/nav/pose`、TF `odom→base_link`、`/nav/loc_valid` |
| `trajectory_node` | 轨迹加载/校验/切换、最近点搜索、误差与进度计算,发布 `/nav/traj/reference` |
| `controller_node` | 50 Hz 跟踪控制律 + 全限幅,输出 `/cmd_vel`;故障/急停强制零速 |
| `exception_node` | 集中异常监测与处置:安全停车 + `/nav/fault`;复位服务 |
| `mission_node` | 对外唯一入口,任务状态机,编排轨迹层与故障联动 |
| `status_node` | 汇聚系统状态发布 `/nav/status`(10 Hz) |
| `sim_node`(tower_nav_sim) | 差速运动学仿真、传感器模拟与异常注入 |

## 2. 分层设计:纯逻辑核心库 + 薄节点封装

所有算法与状态机在**无 ROS 依赖的核心库** `tower_nav_core` 中实现,
节点仅做参数装载、话题/服务绑定与调用转发。收益:gtest 可直接覆盖全部
业务逻辑(111 项测试),节点层无需 mock ROS。

| 库类 | 内容 |
| --- | --- |
| `TowerConfig` | tower.yaml 加载与校验(正值、分段连续、覆盖 0~total_height);r(h) 分段线性插值 |
| `TfProjection` | 表面坐标 (h,s) ↔ 展开坐标 (x,y) ↔ 导航位姿;周向 wrap/unwrap(x mod 2πr(h)) |
| `LocalizationSource` | 定位源抽象基类 + 注册工厂;OdomSource/ImuSource 已实现,Laser/Vision/Uwb 预留 |
| `LocalizationFilter` | 互补滤波融合(IMU 航向为主 α=0.98,编码器角速率预测);里程积分;协方差传播与发散检测 |
| `TrajectoryManager` | 轨迹加载/校验(点距、v_ref)、状态机、最近点/横向误差/航向误差/进度、断点恢复 |
| `TrackingController` | ω = v·κ + k_y·e_y + k_θ·e_θ;速度/加速度/轮速三级限幅;forceStop |
| `ExceptionMonitor` | 心跳超时、定位失效、横向偏差持续超阈、急停、任务超时;故障锁存与复位 |
| `MissionStateMachine` | 任务状态机与命令合法性 |

## 3. 坐标系与塔筒几何

- **展开导航坐标系**:塔筒表面沿母线剪开摊平,x=周向弧长(m)、y=高度(m)、
  yaw=展开面内航向。所有定位、轨迹、控制均在该系内进行。
- **塔筒几何模型**:`tower.yaml` 描述分段锥形(每段底/顶直径线性过渡),
  `TowerConfig::radiusAt(h)` 给出任意高度半径;周长 2πr(h) 随高度变化,
  `TfProjection` 的 wrap/unwrap 据此处理周向环绕。**代码不含任何塔尺寸常量**。
- TF 树:`odom → base_link`(定位);仿真真值独立为 `odom → base_link_gt`,
  互不冲突。

## 4. 状态机

### Mission(mission_node,对外)

```
STANDBY ⇄ READY ─START→ EXECUTING ─PAUSE→ HOLDING
   ▲        ▲             │  ▲  │           │
   │        │(RESET)      │  └──┴─RESUME────┘
   │        │             ├─完成→ COMPLETED ─START/RESET→ …
   │        │             ├─STOP→ ABORTED ──START/RESET→ …
   └────────┴─────────────┴─故障→ ERROR ──RESET(故障已清除)→ READY
```

- STANDBY↔READY 由"系统就绪"(定位有效且无故障)自动迁移;
- START 仅在 READY/COMPLETED/ABORTED 且就绪时接受,轨迹号 1~4 校验;
- **EXECUTING 中 START 一律拒绝**(切换保护);
- ERROR 态仅接受 RESET,且先经 `/nav/fault/reset` 清除异常管理器锁存故障
  (急停未解除时复位失败);RESET 对 READY/STANDBY 幂等。

### Trajectory(trajectory_node,内部)

```
IDLE ─LOAD→ READY ─START→ RUNNING ─PAUSE→ PAUSED ─RESUME→ RUNNING
                    ▲         │到达终点→ FINISHED
                    └──STOP───┴(进度清零)     故障→ FAULT ─RESET→ IDLE
```

PAUSE 保留当前进度索引,RESUME 断点续跑;STOP 清零。加载仅 IDLE/READY/FINISHED
允许(即 Mission 层 READY 语义)。

## 5. 控制律与安全限幅

```
v = v_ref(参考速度)
ω = v·κ (前馈) − [k_y·e_y + k_θ·e_θ](反馈;e_y 左偏为正 → 向右修正)
```

四级安全处理(controller_node,50 Hz):
1. 速度限幅:|v|≤0.30 m/s,|ω|≤0.60 rad/s;
2. 加速度限幅:相对上拍 |dv|≤0.30·dt,|dω|≤1.20·dt;
3. 轮速限幅:换算左右轮速超 5.0 rad/s 时 v/ω 等比缩减;
4. 使能门:故障、急停、参考超时(0.5 s)、跟踪未激活任一成立 → 零速;
   故障/急停走 `forceStop()` 立即归零(不经过加速度斜坡)。

## 6. 异常处理链路

```
异常源 ─▶ ExceptionMonitor.check()(20 Hz,故障锁存,急停最高优先级)
        ─▶ /nav/fault(code, active)+ 安全停车(/cmd_vel 置零)
        ─▶ controller_node:forceStop 强制零速(双保险)
        ─▶ mission_node:notifyFault → ERROR,并 STOP 轨迹层
恢复:排除异常源 → /nav/mission_cmd RESET
        ─▶ mission_node 先调 /nav/fault/reset(急停未解除则失败)
        ─▶ 故障清除 → READY → 重新 START
```

监测项与故障码:IMU/编码器/位姿心跳超时(2/3/4)、定位失效(5)、
横向偏差持续超阈(6,0.30 m 持续 0.3 s,抗瞬时噪声)、急停(7)、任务超时(8)。

## 7. 定位融合

- **航向**:互补滤波。编码器差速角速度积分作高频预测,IMU orientation 作低频
  校正:`yaw = wrap(yaw_pred + α·(yaw_imu − yaw_pred))`,α=0.98;
- **位置**:融合航向 + 编码器线速度中点积分;
- **有效性**:任一源超时(2.0 s)或位置协方差发散(>5 m²)→ `/nav/loc_valid=false`
  → Mission 退 STANDBY、Exception 报 LOC_INVALID;
- **插件架构**:`SourceRegistry` 注册工厂按名创建源;新增激光/视觉/UWB 源仅需
  实现 `LocalizationSource` 接口并注册,融合层无需改动。

## 8. 仿真器(tower_nav_sim)

- 差速运动学中点积分(100 Hz),速度指令立即跟随(执行器动力学不建模,
  加减速由控制器限幅保证);
- 传感器模拟:编码器(高斯噪声)、IMU(姿态噪声 + 陀螺零偏 0.001 rad/s + 噪声),
  固定种子可复现;
- 异常注入:`/sim/imu_enable`(IMU 停发)、`/sim/set_slip`(30% 打滑,
  编码器多计程);
- 真值:`/sim/ground_truth` 与 TF `odom→base_link_gt`。

## 9. 关键设计决策

1. **展开坐标系导航**:把三维曲面导航降为二维平面问题,轨迹/误差/控制全部简化;
   锥度引起的度量畸变在轨迹生成时按 r(h) 折算(见 ASSUMPTIONS #6)。
2. **Mission/Trajectory 双状态机**:对外语义(任务)与对内执行(轨迹)解耦,
   外部只见一个服务接口。
3. **异常集中处置 + 控制器自卫双保险**:exception_node 发零速的同时,
   controller_node 收到故障也独立强制零速,单点失效不失守。
4. **故障锁存**:瞬时异常不会自愈掩盖,必须显式 RESET,符合作业安全规范。
5. **配置驱动**:塔筒几何、控制增益、阈值、频率全部 YAML 参数化。
