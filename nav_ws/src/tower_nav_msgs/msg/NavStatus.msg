# 系统汇总状态(Status Manager 发布于 /nav/status)

# 任务状态枚举(与 Mission Manager 状态机一致)
uint8 MISSION_STANDBY=0
uint8 MISSION_READY=1
uint8 MISSION_EXECUTING=2
uint8 MISSION_HOLDING=3
uint8 MISSION_COMPLETED=4
uint8 MISSION_ABORTED=5
uint8 MISSION_ERROR=6

# 轨迹状态枚举(与 Trajectory Tracking 状态机一致)
uint8 TRAJ_IDLE=0
uint8 TRAJ_READY=1
uint8 TRAJ_RUNNING=2
uint8 TRAJ_PAUSED=3
uint8 TRAJ_FINISHED=4
uint8 TRAJ_FAULT=5

std_msgs/Header header
uint8 mission_state
uint8 traj_state
uint8 traj_id                          # 当前轨迹编号(0 表示未加载)
float64 progress                       # 轨迹进度百分比 0~100
geometry_msgs/PoseWithCovarianceStamped pose   # 实时定位位姿(展开坐标系)
geometry_msgs/Twist velocity           # 当前速度指令
float64 lateral_error                  # 实时横向跟踪误差 m
float64 heading_error                  # 实时航向误差 rad
bool localization_valid                # 定位有效标志
uint16 fault_code                      # 当前故障码(Fault 常量)
