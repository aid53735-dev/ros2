# 部署指南(DEPLOY)

## 1. 环境要求

- Ubuntu 22.04(实机或 WSL2)
- ROS 2 Humble(desktop 或 ros-base + 下述依赖)
- 编译器支持 C++17;colcon、rosdep

安装依赖:

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions \
  python3-rosdep libyaml-cpp-dev python3-yaml \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-ament-cmake-gtest ros-humble-launch-testing-ament-cmake \
  ros-humble-rmw-cyclonedds-cpp
```

## 2. 构建与测试

```bash
cd nav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y   # 可选,补齐依赖
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

要求:构建零错误零警告(全包 `-Wall -Wextra -Werror`);测试全部通过。

## 3. 运行

```bash
source install/setup.bash

# 仿真闭环(默认)
ros2 launch tower_nav bringup.launch.py

# 指定初始位姿(须与所选轨迹起点一致)与轨迹目录
ros2 launch tower_nav bringup.launch.py \
  initial_x:=0.0 initial_y:=20.0 initial_yaw:=0.0 \
  trajectory_dir:=/abs/path/to/trajectories

# 实机(关闭仿真;需实机驱动提供 /wheel/encoder、/imu/data 并订阅 /cmd_vel)
ros2 launch tower_nav bringup.launch.py use_sim:=false
```

验证节点齐全:

```bash
ros2 node list   # 期望 7 个:tower_nav_sim + 6 导航节点(use_sim:=false 时 6 个)
ros2 topic echo /nav/status --once
```

## 4. 任务操作流程

```bash
SRV=/nav/mission_cmd; TYPE=tower_nav_msgs/srv/MissionCommand
ros2 service call $SRV $TYPE "{command: 0, traj_id: 1}"   # START 轨迹 1
ros2 service call $SRV $TYPE "{command: 1}"               # PAUSE(安全驻停)
ros2 service call $SRV $TYPE "{command: 2}"               # RESUME(断点续跑)
ros2 service call $SRV $TYPE "{command: 3}"               # STOP(中止,进度清零)
ros2 service call $SRV $TYPE "{command: 4}"               # RESET(故障复位)
```

进度与状态:`ros2 topic echo /nav/status`(mission_state、progress、
lateral_error、fault_code)。

## 5. 更换塔筒参数(零代码)

1. 新建/修改塔筒配置(如 `config/tower_alt.yaml`):total_height、
   bottom/top_diameter、segments 分段(必须连续覆盖 0~total_height);
2. 重新生成轨迹:
   ```bash
   python3 src/tower_nav/scripts/gen_trajectories.py \
     --tower src/tower_nav/config/tower_alt.yaml \
     --out   src/tower_nav/config/trajectories_alt
   ```
   脚本自校验点距连续性、yaw 与切向一致、v_ref 有效;
3. 以新目录与新起点拉起:
   ```bash
   ros2 launch tower_nav bringup.launch.py \
     trajectory_dir:=$(pwd)/src/tower_nav/config/trajectories_alt \
     initial_y:=30.0
   ```

## 6. 调参要点

| 症状 | 参数 | 方向 |
| --- | --- | --- |
| 转弯切内/横向震荡 | control.yaml `k_y`/`k_theta` | 震荡降增益;偏差大升 k_y |
| 曲线段偏差偏大 | 轨迹 `--v-ref` 或 `max_angular_acc` | 降速或放宽角加速度 |
| 误报心跳超时 | mission.yaml `*_timeout` | 实机低抖动可降至 0.5 s |
| 频繁 LATERAL_DEV | `lateral_dev_threshold`/`lateral_dev_hold` | 按作业容差调整 |
| 定位漂移快 | localization.yaml `heading_alpha` | IMU 好则升,编码器好则降 |

## 7. WSL2 特别注意(开发环境)

1. **RMW 用 CycloneDDS**:`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
   (WSL 下 FastDDS 收包率近 0,详见 ASSUMPTIONS #3);
2. **禁用 systemd-timesyncd**:其在 WSL 内周期性步进墙钟 ±33 s,导致心跳
   超时误判:`sudo systemctl disable --now systemd-timesyncd && sudo systemctl mask systemd-timesyncd`;
3. 源码在 /mnt/c 时构建 IO 慢:用 `tools/wslbuild.sh`(rsync 到 WSL 原生
   文件系统 ~/nav_ws 再构建;`build`/`test`/`run` 三种模式)。

## 8. 实机迁移检查单

- [ ] 驱动板订阅 `/cmd_vel`,发布 `/wheel/encoder`(JointState,velocity[0/1]=左/右轮 rad/s)与 `/imu/data`(orientation 含姿态解算 yaw);
- [ ] `localization.yaml`/`control.yaml`/`sim.yaml` 中 wheel_radius、wheel_separation 与实机一致;
- [ ] tower.yaml 按目标塔实测填写;重新生成轨迹;
- [ ] 心跳超时阈值由 2.0 s 收紧到实机水平(0.5 s);
- [ ] 急停链路:操作端周期发布 `/nav/estop`;验证 ESTOP 触发与复位保护;
- [ ] 首跑用低速轨迹(`--v-ref 0.05`)并全程人工监护。

## 9. 已知验证脚本(tools/)

| 脚本 | 用途 |
| --- | --- |
| `run_all_trajectories.py <out.json> [ids…] [--traj-dir DIR]` | 四轨迹全流程闭环验证(启动/切换保护/暂停零速/断点续跑/完成,记录最大横向误差) |
| `run_exception_injection.py <out.json>` | 三类异常注入验证(IMU 停发/横向偏差/急停 → 安全停车+故障码+复位恢复) |
| `wslbuild.sh {build\|test\|run}` | WSL 构建/测试/运行包装 |
