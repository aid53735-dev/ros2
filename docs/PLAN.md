# 开发计划(PLAN)

> 项目:风机塔筒爬壁机器人 ROS 2 导航定位系统
> 平台:ROS 2 Humble / Ubuntu 22.04 (WSL2) / C++17 / ament_cmake
> 规则:每完成一项 → colcon build 零错误 → 测试通过 → git commit → 勾选进度。

## 任务列表(按依赖排序)

- [ ] **T01 环境准备**
  产出物:WSL Ubuntu-22.04 中可用的 ros2 / colcon / rosdep / gtest / launch_testing;`tools/wslbuild.sh` 构建包装脚本。
  完成判据:`ros2 --help`、`colcon --help` 正常;空工作区 `colcon build` 成功。

- [ ] **T02 工程骨架与 git 初始化**
  产出物:`nav_ws/src/{tower_nav_msgs, tower_nav, tower_nav_sim}` 三包空骨架(package.xml + CMakeLists.txt),docs/PLAN.md、docs/ASSUMPTIONS.md,.gitignore。
  完成判据:`colcon build` 三包零错误零警告;首个 git 提交完成。

- [ ] **T03 tower_nav_msgs 消息与服务定义**
  产出物:TrajectoryPoint.msg、TrackingReference.msg、NavStatus.msg、Fault.msg、MissionCommand.srv、TrajectoryCommand.srv;常量定义(状态枚举、故障码、命令枚举)。
  完成判据:构建通过,`ros2 interface show` 可查看各接口。

- [ ] **T04 TowerConfig 塔筒配置库**
  产出物:`tower_nav::TowerConfig` 类:加载 tower.yaml、合法性校验(正值、分段连续、完整覆盖 0~total_height)、r(h) 分段线性插值;config/tower.yaml 示例(80 m 塔)。
  完成判据:gtest 覆盖合法/非法配置、插值边界与中间值,全部通过。

- [ ] **T05 TFProjection 坐标投影库**
  产出物:`tower_nav::TfProjection`:表面坐标 (h, s) ↔ 展开坐标 (x=s, y=h) ↔ Navigation Pose 双向映射;周向环绕(x mod 2πr(h))处理;由 TowerConfig 驱动。
  完成判据:gtest 双向映射一致性(往返误差 < 1e-9)、不同塔参数下自适应,通过。

- [ ] **T06 tower_nav_sim 仿真节点**
  产出物:差速运动学积分仿真:订阅 /cmd_vel,发布 /wheel/encoder(JointState)、/imu/data(可配噪声/零偏)、地面真值 /sim/ground_truth 与 TF;打滑注入(/sim/set_slip)、IMU 停发注入(/sim/imu_enable);config/sim.yaml。
  完成判据:构建通过;单测验证运动学积分正确(直线、原地转、圆弧)。

- [ ] **T07 LocalizationSource 插件架构**
  产出物:抽象基类 LocalizationSource + 注册工厂;OdomSource(编码器差速解算 v/ω)、ImuSource(姿态/角速度);Laser/Vision/Uwb 预留源(接口完整、注册可用、报告不可用)。
  完成判据:gtest 验证注册/创建流程、OdomSource 差速解算数值正确。

- [ ] **T08 Localization 定位融合节点**
  产出物:localization_node:互补融合(航向 IMU 为主、编码器为辅)、位置里程积分、协方差传播;输出 /nav/pose 与 TF odom→base_link;超时/协方差发散 → 定位失效标志;config/localization.yaml。
  完成判据:gtest 融合数学单测;与 sim 联调:直线 10 m 位置误差 < 2%。

- [ ] **T09 轨迹生成脚本与轨迹文件**
  产出物:scripts/gen_trajectories.py 读取 tower.yaml 生成 4 条轨迹(①定高环向一圈 ②竖直上升 ③之字形覆盖 ④矩形巡检)→ config/trajectories/traj_1~4.yaml(x,y,yaw,v_ref,curvature)。
  完成判据:脚本运行生成 4 文件;点序列连续、曲率/航向与几何一致(脚本自校验)。

- [ ] **T10 Trajectory Tracking 轨迹跟踪节点**
  产出物:trajectory_node:加载/校验轨迹、1~4 切换(仅 READY)、最近点搜索、进度%、到点判定;状态机 IDLE/READY/RUNNING/PAUSED/FINISHED/FAULT;断点恢复;发布 /nav/traj/reference;内部服务 /nav/traj/cmd。
  完成判据:gtest 状态机迁移全覆盖 + 最近点/进度计算正确。

- [ ] **T11 Navigation Controller 导航控制器节点**
  产出物:controller_node:ω = v·κ + k_y·e_y + k_θ·e_θ;50 Hz 输出 /cmd_vel;限幅(线速/角速/加速度/轮速);FAULT/急停强制零速;config/control.yaml。
  完成判据:gtest 误差求解(e_y、e_θ 符号与数值)、限幅逻辑单测通过。

- [ ] **T12 Exception Manager 异常管理节点**
  产出物:exception_node:话题心跳超时(IMU/编码器/位姿)、定位失效、横向偏差超阈、急停 /nav/estop、任务超时监测;安全停车 + /nav/fault 上报;复位服务 /nav/fault/reset。
  完成判据:gtest 各触发条件与复位逻辑单测通过。

- [ ] **T13 Mission Manager 任务管理节点**
  产出物:mission_node:服务 /nav/mission_cmd(START/PAUSE/RESUME/STOP/RESET+轨迹号);状态机 STANDBY/READY/EXECUTING/HOLDING/COMPLETED/ABORTED/ERROR;编排 trajectory 与 controller;故障联动。
  完成判据:gtest 状态机与命令合法性迁移全覆盖。

- [ ] **T14 Status Manager 状态节点**
  产出物:status_node:汇聚 /nav/status(机器人状态、轨迹号与进度、任务状态、Pose、速度、跟踪误差、故障码)。
  完成判据:构建通过;联调时 /nav/status 字段齐全且更新。

- [ ] **T15 Launch 与参数体系**
  产出物:bringup.launch.py(use_sim 参数);分域 YAML:tower/localization/control/mission/trajectories;sim.yaml。
  完成判据:一键拉起全部节点,`ros2 node list` 七节点齐全,无崩溃。

- [ ] **T16 launch_testing 集成测试**
  产出物:test/integration/test_mission_flow.py:sim+全栈 → START 轨迹1 → 断言最大横向误差<阈值、PAUSE 速度为零、RESUME 继续、急停后 /cmd_vel=0 且故障码正确。
  完成判据:`colcon test` 集成测试通过。

- [ ] **T17 四条轨迹全流程闭环验证**
  产出物:tools/run_all_trajectories.sh 验证脚本 + 数据记录。
  完成判据:4 条轨迹均完成 启动→跟踪→完成;切换/暂停/恢复/停止可用;记录各轨迹最大横向误差。

- [ ] **T18 塔筒参数更换验证**
  产出物:config/tower_alt.yaml(120 m、直径与分段数不同);重新生成轨迹并跑通。
  完成判据:不改任何代码,轨迹正确映射且稳定跟踪。

- [ ] **T19 异常注入验证**
  产出物:异常注入验证记录(停发 IMU、超阈偏差、急停)。
  完成判据:三种异常均安全停车 + 正确故障码;复位后可恢复作业。

- [ ] **T20 文档交付**
  产出物:docs/README.md、ARCHITECTURE.md、INTERFACE.md、DEPLOY.md。
  完成判据:四份文档齐全、与实现一致。

- [ ] **T21 验收报告与最终迭代**
  产出物:docs/ACCEPTANCE_REPORT.md,逐条记录五项验收标准实测数据。
  完成判据:五项验收标准全部达标(含 colcon build 零警告、colcon test 全过)。
