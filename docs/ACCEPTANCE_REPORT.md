# 验收报告(ACCEPTANCE_REPORT)

> 项目:风机塔筒爬壁机器人 ROS 2 导航定位系统
> 验收日期:2026-07-18
> 环境:ROS 2 Humble / Ubuntu 22.04(WSL2)/ C++17 / ament_cmake / RMW=CycloneDDS
> 结论:**五项验收标准全部达标**。

---

## 验收项 1:构建零错误零警告 + 全部测试通过 ✅

- 全部三包(tower_nav_msgs、tower_nav、tower_nav_sim)开启
  `-Wall -Wextra -Werror`,任何警告即构建失败;
- 最终复跑命令:
  ```
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  colcon test --return-code-on-test-failure && colcon test-result --verbose
  ```

实测:

| 指标 | 结果 |
| --- | --- |
| colcon build | 3 包全部成功,0 错误 0 警告 |
| colcon test | **111 项测试,0 失败 0 错误**(测试套件明细见下) |

测试套件构成:

| 套件 | 覆盖 |
| --- | --- |
| test_tower_config | tower.yaml 合法/非法配置、r(h) 插值边界与中间值 |
| test_tf_projection | 表面↔展开双向映射(往返 <1e-9)、wrap/unwrap、80m/120m 塔自适应 |
| test_localization_source | 插件注册工厂、OdomSource 差速解算、预留源不可用报告 |
| test_localization_filter | 互补融合数学、超时/协方差发散失效判定 |
| test_trajectory_manager | 轨迹加载校验、状态机全迁移、最近点/误差/进度、断点恢复 |
| test_tracking_controller | 控制律符号与数值、速度/加速度/轮速限幅、forceStop |
| test_exception_monitor | 各故障触发条件、锁存、优先级、急停复位保护 |
| test_mission_state_machine | 任务状态机与命令合法性全覆盖(含 RESET 幂等) |
| test_mission_flow(launch_testing 集成) | sim+全栈:START→跟踪误差阈值→PAUSE 零速→RESUME→急停→复位 |
| test_diff_drive_sim | 运动学积分(直线/原地转/圆弧)、噪声/打滑注入 |

## 验收项 2:四条轨迹全流程闭环 ✅

脚本 `tools/run_all_trajectories.py`,每条轨迹独立拉起全栈
(`bringup.launch.py`,初始位姿=轨迹起点),实测数据
`docs/data/t17_trajectories.json`:

| 轨迹 | 描述 | 长度 m | 最大横向误差 m | 用时 s | 完成 |
| --- | --- | --- | --- | --- | --- |
| 1 | 定高环向一圈 h=20.0 m | 12.36 | **0.0001** | 93.0 | ✅ COMPLETED |
| 2 | 竖直上升 8→48 m | 40.00 | **0.0001** | 277.4 | ✅ COMPLETED |
| 3 | 之字形覆盖 12~36 m(4 行) | 58.50 | **0.2155** | 406.7 | ✅ COMPLETED |
| 4 | 矩形巡检 5.9×16 m | 43.79 | **0.2148** | 304.9 | ✅ COMPLETED |

> 轨迹 3/4 的峰值误差出现在 90° 直角拐点(角加速度限幅所致的过渡弧),
> 直线/环向段误差与轨迹 1/2 同级(<0.001 m),且始终低于 0.30 m 故障阈值。

任务管理功能实测(同脚本,轨迹 1 附加检查):

| 功能 | 实测结果 |
| --- | --- |
| 切换保护 | EXECUTING 中 START 轨迹 2 被拒:"START 非法:当前状态 EXECUTING" |
| 停止 | STOP → ABORTED,进度清零 |
| 重启 | ABORTED 后重新 START 正常 |
| 暂停零速 | PAUSE 后采样 1.5 s:\|v\|max=0.00e+00,\|w\|max=0.00e+00 |
| 断点续跑 | RESUME 从 25.0% 断点继续推进至完成 |

## 验收项 3:塔筒参数更换零代码 ✅

- 新配置 `config/tower_alt.yaml`:**120 m、四段**锥形(直径 6.0→3.0),
  与默认 80 m 三段塔的总高/直径/分段数均不同;
- 流程:`gen_trajectories.py --tower tower_alt.yaml` 重新生成 4 条轨迹
  (脚本自校验通过)→ `bringup.launch.py trajectory_dir:=…` 切换目录;
- **全程未修改任何 C++/Python/launch 代码**(仅新增配置文件与数据);
- 实测数据 `docs/data/t18_trajectories_alt.json`:

| 轨迹 | 描述(120 m 塔自适应) | 长度 m | 最大横向误差 m | 用时 s | 完成 |
| --- | --- | --- | --- | --- | --- |
| 1 | 定高环向一圈 h=30.0 m(周长 16.96 m) | 16.97 | 0.0001 | 123.8 | ✅ |
| 2 | 竖直上升 12→72 m | 60.00 | 0.0001 | 410.7 | ✅ |
| 3 | 之字形覆盖 18~54 m 宽 9.1 m | 81.49 | 0.2136 | 559.9 | ✅ |
| 4 | 矩形巡检 7.8×24 m | 63.67 | 0.2151 | 437.4 | ✅ |

轨迹几何随塔参数正确缩放(环向周长 12.36→16.96 m 对应 r(20)→r(30) 变化;
覆盖带宽 6.9→9.1 m),映射与跟踪稳定性与 80 m 塔一致。

## 验收项 4:异常注入 → 安全停车 + 正确故障码 + 复位恢复 ✅

脚本 `tools/run_exception_injection.py`,实测数据 `docs/data/t19_exceptions.json`:

| 场景 | 注入方式 | 故障码 | 安全停车 | 复位恢复 |
| --- | --- | --- | --- | --- |
| 停发 IMU | `/sim/imu_enable false` | **2(IMU_TIMEOUT)**,任务进 ERROR | \|v\|max=\|w\|max=0.00e+00 | 恢复 IMU → RESET → 重新 START 跟踪恢复 ✅ |
| 横向偏差超阈 | 对抗性 /cmd_vel 扰动推离轨迹 | **6(LATERAL_DEV)**,触发时 \|e_y\|=0.489 m(阈值 0.30 m 持续 0.3 s),任务进 ERROR | \|v\|max=\|w\|max=0.00e+00 | RESET + 遥控归位 → 重新 START 恢复作业 ✅ |
| 急停 | `/nav/estop true` | **7(ESTOP)**,任务进 ERROR | \|v\|max=\|w\|max=0.00e+00 | **急停未解除时 RESET 被拒**("复位失败:故障未清除");解除后 RESET → 重新 START 恢复 ✅ |

3/3 场景通过;故障锁存、急停最高优先级与复位保护行为均符合设计。

## 验收项 5:文档交付 ✅

| 文档 | 内容 | 状态 |
| --- | --- | --- |
| docs/README.md | 项目说明、功能特性、包结构、快速开始、换塔/异常注入操作 | ✅ |
| docs/ARCHITECTURE.md | 节点拓扑、核心库分层、双状态机、控制律、异常链路、设计决策 | ✅ |
| docs/INTERFACE.md | 全部话题/服务/消息/故障码/轨迹格式/参数规格 | ✅ |
| docs/DEPLOY.md | 环境、构建、运行、调参、WSL 注意事项、实机迁移检查单 | ✅ |
| docs/ACCEPTANCE_REPORT.md | 本报告(逐项实测数据) | ✅ |
| docs/PLAN.md / ASSUMPTIONS.md | 计划(T01~T21 全部勾选)与 21 条工程假设 | ✅ |

---

## 附:验证方法与可复现性

- 所有实测数据由 `tools/` 下脚本自动采集并以 JSON 存档于 `docs/data/`;
- 仿真噪声固定种子(42),结果可复现;
- 验证环境注意事项(CycloneDDS、禁用 timesyncd)见 ASSUMPTIONS #3/#4 与
  DEPLOY §7;
- 提交历史遵循 Conventional Commits,每任务一提交,构建/测试通过后提交。
