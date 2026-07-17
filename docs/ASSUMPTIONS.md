# 工程假设(ASSUMPTIONS)

> 无人值守模式下,遇不明确处按以下工程假设推进。

## 环境
1. **宿主环境为 Windows,实际构建/运行在 WSL2 Ubuntu-22.04 中**。任务书要求 ROS 2 Humble + Ubuntu 22.04,Windows 侧无 ROS;WSL2 内已安装 Ubuntu 22.04.5,故所有 colcon build / test / 运行均通过 `wsl -d Ubuntu-22.04` 执行。仓库位于 Windows 侧 `C:\Users\ASUS\Desktop\ros2`(WSL 路径 `/mnt/c/Users/ASUS/Desktop/ros2`)。为避免 /mnt/c 跨文件系统 IO 过慢,构建产物(build/install/log)放在 WSL 原生文件系统 `~/nav_ws` 下,源码经 rsync 同步(见 tools/wslbuild.sh)。
2. WSL 中以 root 安装系统依赖(普通用户 sudo 需密码,无人值守不可交互输入)。
3. **RMW 实现固定为 CycloneDDS**:WSL2 下默认 FastDDS 传输不可靠(实测 100 Hz 话题订阅端收包率近 0),`tools/wslbuild.sh` 统一导出 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。
4. **禁用 WSL 内 systemd-timesyncd**:实测其周期性把系统墙钟往复步进 ±32.9 s(每 ~25 s 一次),导致 ROS 时间(默认墙钟)跳变、心跳超时误判(集成测试随机报 IMU/编码器超时、`ros2 topic hz` 出现 32 s 间隔)。已 `systemctl disable --now && systemctl mask systemd-timesyncd`;时间同步交由 WSL 宿主机制处理。

## 机器人与运动学
5. **机器人模型**:双轮差速,轮距 `wheel_separation = 0.40 m`,轮半径 `wheel_radius = 0.08 m`(磁吸附爬壁机器人典型量级)。参数入 YAML,可改。
6. **导航平面**:机器人贴塔筒表面运动,导航在二维展开坐标系 (x=周向弧长, y=高度) 中进行,重力/吸附动力学不建模(定位导航层不需要)。展开坐标系下按平面差速运动学处理;锥度引起的度量畸变在轨迹生成时按参考高度的 r(h) 折算,跟踪层不再修正(锥角小,误差可忽略;记入文档)。
7. **周向环绕**:x 方向在展开面上是周期的(周长 = 2πr(h))。TF Projection 提供 wrap/unwrap;轨迹与定位内部使用连续(unwrapped)x,便于误差计算。
8. **IMU 航向**:仿真 IMU 提供 orientation(积分 yaw + 可配零偏与噪声),模拟实际 IMU 内置姿态解算输出;定位节点航向以 IMU orientation 为主、编码器 yaw 速率为辅做互补滤波。

## 坐标系
9. TF 树:`odom → base_link`(定位节点发布);仿真另发布 `odom → base_link_gt` 用于评估(地面真值走独立话题 /sim/ground_truth,不与定位 TF 冲突)。展开坐标系原点:h=0、s=0 处,x 轴沿周向、y 轴沿高度。
10. 机器人初始位姿:默认位于轨迹起点附近(由 launch 参数指定),yaw 沿轨迹初始方向。

## 接口与行为
11. **状态机对齐**:任务书中 Trajectory 状态机(IDLE/READY/RUNNING/PAUSED/FINISHED/FAULT)与 Mission 状态机(STANDBY/READY/EXECUTING/HOLDING/COMPLETED/ABORTED/ERROR)独立实现,Mission 为对外唯一入口,Trajectory 的服务仅供 Mission 内部调用。
12. **轨迹切换**:仅 Mission=READY(对应 Trajectory=READY 或 IDLE)时允许 START 指定新轨迹编号;RUNNING 中 START 返回失败。
13. **断点恢复**:PAUSE 记录当前进度索引,RESUME 从该索引继续;STOP 清零进度回 READY。
14. **急停**:/nav/estop (std_msgs/Bool, latched 行为按周期发布) true → Exception Manager 立即置故障 ESTOP,控制器强制零速;复位需先解除 estop 再调 /nav/fault/reset。
15. **故障码**:uint16 枚举定义在 tower_nav_msgs(NONE=0, CONFIG_INVALID, IMU_TIMEOUT, ENCODER_TIMEOUT, POSE_TIMEOUT, LOC_INVALID, LATERAL_DEV, ESTOP, MISSION_TIMEOUT, TRAJ_INVALID)。
16. **心跳超时阈值**:IMU/编码器/pose 均 2.0 s(WSL 调度抖动裕量;实机可下调至 0.5 s);横向偏差阈值 0.30 m;任务超时默认 1200 s。均可配。
17. **控制频率** 50 Hz;仿真步率 100 Hz;定位输出 50 Hz;状态发布 10 Hz。
18. **验收阈值**:集成测试最大横向误差阈值 0.15 m(轨迹1 定高环向,标称速度 0.15 m/s);"直线 10 m 误差 < 2%" 为无噪声情形。

## 质量
19. 单元测试在无 ROS 运行时依赖的纯逻辑层(库类)上做;节点薄封装。
20. `-Wall -Wextra -Werror` 全包开启(第三方头文件警告用 isystem 隔离)。
21. 注释与文档中文,标识符英文;commit message 采用 Conventional Commits(英文 subject,中文 body 允许)。
