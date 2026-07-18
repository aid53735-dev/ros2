# 接口规格(INTERFACE)

## 1. 话题

| 话题 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `/cmd_vel` | geometry_msgs/Twist | controller_node、exception_node(安全停车) | sim_node | 50 Hz | 速度指令(linear.x, angular.z) |
| `/wheel/encoder` | sensor_msgs/JointState | sim_node | localization_node、exception_node | 100 Hz | 左右轮角度/角速度(name: left/right_wheel_joint) |
| `/imu/data` | sensor_msgs/Imu | sim_node | localization_node、exception_node | 100 Hz | orientation(姿态解算 yaw)+ angular_velocity.z |
| `/sim/ground_truth` | nav_msgs/Odometry | sim_node | (评估用) | 100 Hz | 地面真值位姿与速度 |
| `/nav/pose` | geometry_msgs/PoseWithCovarianceStamped | localization_node | trajectory_node、exception_node、status_node | 50 Hz | 融合定位(展开坐标系,frame_id=odom) |
| `/nav/loc_valid` | std_msgs/Bool | localization_node | mission_node、exception_node、status_node | 50 Hz | 定位有效标志 |
| `/nav/traj/reference` | tower_nav_msgs/TrackingReference | trajectory_node | controller_node、exception_node、mission_node、status_node | 随 /nav/pose | 跟踪参考 + 误差 + 进度 |
| `/nav/mission_state` | std_msgs/UInt8 | mission_node | status_node、外部 | 10 Hz | 任务状态枚举(NavStatus MISSION_*) |
| `/nav/fault` | tower_nav_msgs/Fault | exception_node | controller_node、mission_node、status_node | 20 Hz | 故障码与激活标志 |
| `/nav/estop` | std_msgs/Bool | 外部(操作端) | exception_node、controller_node | 按需(建议 ≥10 Hz 周期发布) | 急停 |
| `/nav/status` | tower_nav_msgs/NavStatus | status_node | 外部 | 10 Hz | 系统汇总状态 |
| TF `odom→base_link` | tf2 | localization_node | — | 50 Hz | 定位位姿 |
| TF `odom→base_link_gt` | tf2 | sim_node | — | 100 Hz | 仿真真值(独立帧) |

## 2. 服务

### `/nav/mission_cmd`(tower_nav_msgs/srv/MissionCommand)——对外唯一控制入口

请求:
| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `command` | uint8 | START=0 / PAUSE=1 / RESUME=2 / STOP=3 / RESET=4 |
| `traj_id` | uint8 | 仅 START 有效:轨迹编号 1~4 |

响应:`success`(bool)、`message`(string,中文)、`mission_state`(uint8)。

命令合法性(非法一律 `success=false` 并给出原因):

| 命令 | 允许的当前状态 | 结果状态 |
| --- | --- | --- |
| START | READY / COMPLETED / ABORTED(且系统就绪,traj_id∈[1,4]) | EXECUTING |
| PAUSE | EXECUTING | HOLDING(安全驻停,速度归零) |
| RESUME | HOLDING | EXECUTING(断点续跑) |
| STOP | EXECUTING / HOLDING | ABORTED(进度清零) |
| RESET | ERROR / ABORTED / COMPLETED / READY / STANDBY(幂等) | READY(或未就绪时 STANDBY);ERROR 时先清异常故障,急停未解除则失败 |

### `/nav/traj/cmd`(tower_nav_msgs/srv/TrajectoryCommand)——内部,仅 Mission 调用

`command`:LOAD=0(带 traj_id)/ START=1 / PAUSE=2 / RESUME=3 / STOP=4 / RESET=5。
响应含 `traj_state`(TRAJ_* 枚举)。

### `/nav/fault/reset`(std_srvs/srv/Trigger)

清除异常管理器锁存故障。急停仍激活时返回 `success=false`。

### 仿真注入(tower_nav_sim)

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/sim/imu_enable` | std_srvs/SetBool | false=停发 IMU(异常注入),true=恢复 |
| `/sim/set_slip` | std_srvs/SetBool | true=注入 30% 打滑(编码器多计程),false=关闭 |

## 3. 消息定义(tower_nav_msgs)

### TrajectoryPoint.msg
```
float64 x          # 周向弧长 m(展开坐标系)
float64 y          # 高度 m
float64 yaw        # 展开面内航向 rad
float64 v_ref      # 参考线速度 m/s
float64 curvature  # 参考曲率 1/m
```

### TrackingReference.msg
```
std_msgs/Header header
tower_nav_msgs/TrajectoryPoint point   # 最近参考点
float64 lateral_error    # e_y m,左偏为正
float64 heading_error    # e_theta rad
float64 progress         # 0~100 %
uint8 traj_id            # 1~4
bool tracking_active     # RUNNING 态 true;false 时控制器输出零速
```

### NavStatus.msg
```
uint8 MISSION_STANDBY=0 READY=1 EXECUTING=2 HOLDING=3 COMPLETED=4 ABORTED=5 ERROR=6
uint8 TRAJ_IDLE=0 READY=1 RUNNING=2 PAUSED=3 FINISHED=4 FAULT=5
std_msgs/Header header
uint8 mission_state
uint8 traj_state
uint8 traj_id
float64 progress
geometry_msgs/PoseWithCovarianceStamped pose
geometry_msgs/Twist velocity
float64 lateral_error
float64 heading_error
bool localization_valid
uint16 fault_code
```

### Fault.msg 与故障码
```
std_msgs/Header header
uint16 code
string description   # 中文描述
bool active
```

| 码 | 名称 | 触发条件 | 处置 |
| --- | --- | --- | --- |
| 0 | NONE | — | — |
| 1 | CONFIG_INVALID | 塔筒/轨迹配置校验失败 | 拒绝加载 |
| 2 | IMU_TIMEOUT | /imu/data 超 2.0 s 无数据 | 安全停车+锁存 |
| 3 | ENCODER_TIMEOUT | /wheel/encoder 超 2.0 s | 安全停车+锁存 |
| 4 | POSE_TIMEOUT | /nav/pose 超 2.0 s | 安全停车+锁存 |
| 5 | LOC_INVALID | 定位失效(源超时/协方差发散) | 安全停车+锁存 |
| 6 | LATERAL_DEV | \|e_y\|>0.30 m 持续 0.3 s | 安全停车+锁存 |
| 7 | ESTOP | /nav/estop true(最高优先级,抢占) | 立即零速;未解除禁止复位 |
| 8 | MISSION_TIMEOUT | 任务执行超 1200 s | 安全停车+锁存 |
| 9 | TRAJ_INVALID | 轨迹加载/校验失败 | 拒绝启动 |

## 4. 轨迹文件格式(config/trajectories/traj_N.yaml)

```yaml
trajectory:
  id: 1
  description: "定高环向一圈 h=20.0m 周长=12.36m"
  points:                 # 等距重采样(0.05 m),点距>1.0 m 判非法
    - {x: 0.0, y: 20.0, yaw: 0.0, v_ref: 0.15, curvature: 0.0}
    - …
```

## 5. 主要参数(YAML)

| 文件 | 关键参数(默认) |
| --- | --- |
| `tower.yaml` | total_height=80、分段 segments[](start/end_height、bottom/top_diameter) |
| `localization.yaml` | wheel_radius=0.08、wheel_separation=0.40、heading_alpha=0.98、sensor_timeout=2.0、cov_diverge_threshold=5.0、output_rate=50、initial_x/y/yaw |
| `control.yaml` | k_y=1.2、k_theta=1.8、max_linear_vel=0.30、max_angular_vel=0.60、max_linear_acc=0.30、max_angular_acc=1.20、max_wheel_vel=5.0、control_rate=50、reference_timeout=0.5 |
| `mission.yaml` | imu/encoder/pose_timeout=2.0、lateral_dev_threshold=0.30、lateral_dev_hold=0.3、mission_timeout=1200、startup_grace=3.0、max_point_gap=1.0、goal_tolerance=0.15 |
| `sim.yaml` | 噪声/零偏/种子、step_rate=100、initial_x/y/yaw |
| launch 参数 | use_sim(true)、trajectory_dir、initial_x/y/yaw(透传 sim 与定位) |
