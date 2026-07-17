// 轨迹管理核心(纯逻辑,可单测):
// 轨迹加载与校验、状态机(IDLE/READY/RUNNING/PAUSED/FINISHED/FAULT)、
// 最近点搜索、进度统计、到点判定、断点恢复。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace tower_nav
{

/// 轨迹离散点(与 tower_nav_msgs/TrajectoryPoint 对应)
struct TrajPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v_ref{0.0};
  double curvature{0.0};
};

/// 轨迹状态机状态
enum class TrajState
{
  kIdle = 0,
  kReady = 1,
  kRunning = 2,
  kPaused = 3,
  kFinished = 4,
  kFault = 5,
};

const char * toString(TrajState s);

/// 跟踪参考输出
struct TrackingRef
{
  TrajPoint point;          // 最近参考点
  double lateral_error{0.0};   // 横向误差 e_y(m,参考航向左侧为正)
  double heading_error{0.0};   // 航向误差 e_θ(rad)
  double progress{0.0};        // 进度百分比 0~100
  bool goal_reached{false};    // 到达终点
};

/// 轨迹管理器
class TrajectoryManager
{
public:
  /// 校验参数
  struct Limits
  {
    double max_point_gap{1.0};       // 相邻点最大间距 m
    double goal_tolerance{0.15};     // 到点判定距离 m
    size_t min_points{2};            // 最少点数
  };

  explicit TrajectoryManager(const Limits & limits);

  /// 从 YAML 文件加载轨迹(traj_N.yaml);失败返回 false 并置错误信息
  bool loadFromFile(uint8_t traj_id, const std::string & path);

  /// 直接加载点序列(便于测试);校验失败返回 false
  bool loadPoints(uint8_t traj_id, const std::vector<TrajPoint> & points);

  // ---- 状态机操作:非法迁移返回 false ----
  bool start();    // READY→RUNNING;FINISHED→(重置进度)RUNNING
  bool pause();    // RUNNING→PAUSED
  bool resume();   // PAUSED→RUNNING(断点恢复:保留进度索引)
  bool stop();     // RUNNING/PAUSED→READY(进度清零)
  bool reset();    // FAULT→IDLE(清空轨迹)
  void fault();    // 任意态→FAULT

  /// 由当前位姿计算跟踪参考(仅 RUNNING 有效;到达终点自动 FINISHED)
  TrackingRef update(double x, double y, double yaw);

  TrajState state() const {return state_;}
  uint8_t trajId() const {return traj_id_;}
  double progress() const {return progress_;}
  size_t currentIndex() const {return current_index_;}
  const std::vector<TrajPoint> & points() const {return points_;}
  const std::string & lastError() const {return last_error_;}

private:
  bool validate(const std::vector<TrajPoint> & points);
  static double wrapAngle(double a);

  Limits limits_;
  TrajState state_{TrajState::kIdle};
  uint8_t traj_id_{0};
  std::vector<TrajPoint> points_;
  std::vector<double> cum_arc_;   // 各点累计弧长(进度统计用)
  size_t current_index_{0};
  double progress_{0.0};
  std::string last_error_;
};

}  // namespace tower_nav
