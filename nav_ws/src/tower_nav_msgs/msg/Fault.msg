# 系统故障码(Exception Manager 发布于 /nav/fault)
uint16 NONE=0
uint16 CONFIG_INVALID=1      # 塔筒/轨迹配置校验失败
uint16 IMU_TIMEOUT=2         # /imu/data 超时
uint16 ENCODER_TIMEOUT=3     # /wheel/encoder 超时
uint16 POSE_TIMEOUT=4        # /nav/pose 超时
uint16 LOC_INVALID=5         # 定位失效(协方差发散等)
uint16 LATERAL_DEV=6         # 横向偏差超阈
uint16 ESTOP=7               # 急停触发
uint16 MISSION_TIMEOUT=8     # 任务超时
uint16 TRAJ_INVALID=9        # 轨迹加载/校验失败

std_msgs/Header header
uint16 code
string description           # 故障描述(中文)
bool active                  # 故障是否处于激活态
