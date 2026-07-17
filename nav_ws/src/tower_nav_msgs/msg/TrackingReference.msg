# 轨迹跟踪节点发布给控制器的参考量
std_msgs/Header header
tower_nav_msgs/TrajectoryPoint point   # 最近参考点
float64 lateral_error                  # 横向误差 e_y(m,左正)
float64 heading_error                  # 航向误差 e_theta(rad)
float64 progress                       # 进度百分比 0~100
uint8 traj_id                          # 当前轨迹编号 1~4
bool tracking_active                   # RUNNING 态为 true,控制器据此输出/停车
