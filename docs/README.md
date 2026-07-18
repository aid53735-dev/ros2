# 风机塔筒爬壁机器人 ROS 2 导航定位系统

面向风机塔筒表面作业(检测/清洗/维护)的爬壁机器人导航定位软件栈。
机器人磁吸附于塔筒外表面,在**塔筒展开二维坐标系**(x=周向弧长 m,y=高度 m)
内完成定位融合、轨迹跟踪、任务管理与异常保护;配套差速运动学仿真器可在无实机
条件下全流程闭环验证。

- 平台:ROS 2 Humble / Ubuntu 22.04 / C++17 / ament_cmake
- 质量:全包 `-Wall -Wextra -Werror` 零警告;gtest 单元测试 + launch_testing 集成测试

## 功能特性

| 能力 | 说明 |
| --- | --- |
| 塔筒参数化 | `tower.yaml` 描述总高/分段锥形直径,r(h) 分段线性插值;**换塔只改配置,零代码修改** |
| 定位融合 | 编码器差速解算 + IMU 航向互补滤波;插件式定位源架构(Laser/Vision/UWB 预留) |
| 轨迹跟踪 | 4 条参数化轨迹(定高环向/竖直上升/之字形覆盖/矩形巡检);暂停-断点续跑;进度上报 |
| 导航控制 | ω = v·κ + k_y·e_y + k_θ·e_θ 前馈+反馈,50 Hz;线速/角速/加速度/轮速全限幅 |
| 任务管理 | 唯一对外入口 `/nav/mission_cmd`;STANDBY/READY/EXECUTING/HOLDING/COMPLETED/ABORTED/ERROR |
| 异常保护 | 心跳超时/定位失效/横向偏差超阈/急停/任务超时 → 安全停车 + 故障码;复位可恢复 |
| 仿真验证 | 差速运动学积分 + 传感器噪声/零偏;IMU 停发、轮子打滑注入接口 |

## 包结构

```
nav_ws/src/
├── tower_nav_msgs/    # 消息与服务接口(TrajectoryPoint/TrackingReference/NavStatus/Fault,
│                      #   MissionCommand/TrajectoryCommand)
├── tower_nav/         # 导航主包:核心库 + 六节点 + 配置 + 轨迹 + 测试
│   ├── include/…      # TowerConfig / TfProjection / LocalizationSource / LocalizationFilter /
│   │                  #   TrajectoryManager / TrackingController / ExceptionMonitor / MissionStateMachine
│   ├── src/…          # localization_node / trajectory_node / controller_node /
│   │                  #   exception_node / mission_node / status_node
│   ├── config/        # tower.yaml、tower_alt.yaml、localization/control/mission.yaml、trajectories*/
│   ├── scripts/       # gen_trajectories.py(由 tower.yaml 生成 4 条轨迹)
│   ├── launch/        # bringup.launch.py
│   └── test/          # gtest ×8 + launch_testing 集成测试
└── tower_nav_sim/     # 差速运动学仿真节点(噪声/零偏/打滑/IMU 停发注入)
```

## 快速开始

```bash
# 1. 构建(工作区根目录)
cd nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. 测试(111 项:gtest 单测 + 集成测试)
colcon test --return-code-on-test-failure && colcon test-result --verbose

# 3. 一键拉起全栈(含仿真)
source install/setup.bash
ros2 launch tower_nav bringup.launch.py            # use_sim:=false 可关闭仿真

# 4. 启动任务:轨迹 1(定高环向一圈)
ros2 service call /nav/mission_cmd tower_nav_msgs/srv/MissionCommand "{command: 0, traj_id: 1}"

# 暂停 / 恢复 / 停止 / 复位
ros2 service call /nav/mission_cmd tower_nav_msgs/srv/MissionCommand "{command: 1}"  # PAUSE
ros2 service call /nav/mission_cmd tower_nav_msgs/srv/MissionCommand "{command: 2}"  # RESUME
ros2 service call /nav/mission_cmd tower_nav_msgs/srv/MissionCommand "{command: 3}"  # STOP
ros2 service call /nav/mission_cmd tower_nav_msgs/srv/MissionCommand "{command: 4}"  # RESET

# 5. 观察状态
ros2 topic echo /nav/status
```

## 更换塔筒(零代码)

```bash
# 编辑/替换 config/tower.yaml(或直接用 config/tower_alt.yaml,120 m 四段塔)
python3 scripts/gen_trajectories.py --tower config/tower_alt.yaml --out config/trajectories_alt
ros2 launch tower_nav bringup.launch.py trajectory_dir:=<trajectories_alt 绝对路径> \
  initial_y:=30.0   # 初始位姿按新轨迹起点
```

## 异常注入(仿真)

```bash
ros2 service call /sim/imu_enable std_srvs/srv/SetBool "{data: false}"  # 停发 IMU
ros2 service call /sim/set_slip  std_srvs/srv/SetBool "{data: true}"    # 30% 打滑
ros2 topic pub /nav/estop std_msgs/msg/Bool "{data: true}" -r 10        # 急停
```

任一异常触发后:控制器立即零速、`/nav/fault` 上报故障码、任务进 ERROR;
排除异常源后 `RESET` 复位即可重新作业。

## 文档

- [ARCHITECTURE.md](ARCHITECTURE.md) —— 架构设计:节点拓扑、核心库、状态机、坐标系
- [INTERFACE.md](INTERFACE.md) —— 接口规格:话题/服务/消息/参数/故障码
- [DEPLOY.md](DEPLOY.md) —— 部署指南:环境准备、构建、运行、调参、实机迁移
- [ACCEPTANCE_REPORT.md](ACCEPTANCE_REPORT.md) —— 验收报告(实测数据)
- [PLAN.md](PLAN.md) / [ASSUMPTIONS.md](ASSUMPTIONS.md) —— 开发计划与工程假设
