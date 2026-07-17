// 轨迹管理核心实现。
#include "tower_nav/trajectory_manager.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace tower_nav
{

const char * toString(TrajState s)
{
  switch (s) {
    case TrajState::kIdle: return "IDLE";
    case TrajState::kReady: return "READY";
    case TrajState::kRunning: return "RUNNING";
    case TrajState::kPaused: return "PAUSED";
    case TrajState::kFinished: return "FINISHED";
    case TrajState::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

TrajectoryManager::TrajectoryManager(const Limits & limits)
: limits_(limits)
{
}

double TrajectoryManager::wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

bool TrajectoryManager::loadFromFile(uint8_t traj_id, const std::string & path)
{
  std::vector<TrajPoint> pts;
  try {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root["trajectory"] || !root["trajectory"]["points"]) {
      last_error_ = "轨迹文件缺少 trajectory.points: " + path;
      return false;
    }
    for (const auto & p : root["trajectory"]["points"]) {
      TrajPoint tp;
      tp.x = p["x"].as<double>();
      tp.y = p["y"].as<double>();
      tp.yaw = p["yaw"].as<double>();
      tp.v_ref = p["v_ref"].as<double>();
      tp.curvature = p["curvature"].as<double>();
      pts.push_back(tp);
    }
  } catch (const YAML::Exception & e) {
    last_error_ = std::string("轨迹文件解析失败: ") + e.what();
    return false;
  }
  return loadPoints(traj_id, pts);
}

bool TrajectoryManager::loadPoints(
  uint8_t traj_id, const std::vector<TrajPoint> & points)
{
  // 仅 IDLE / READY / FINISHED 态允许加载(切换轨迹)
  if (state_ != TrajState::kIdle && state_ != TrajState::kReady &&
    state_ != TrajState::kFinished)
  {
    last_error_ = std::string("当前状态不允许加载轨迹: ") + toString(state_);
    return false;
  }
  if (!validate(points)) {
    return false;   // last_error_ 已置
  }
  points_ = points;
  traj_id_ = traj_id;
  // 预计算累计弧长
  cum_arc_.assign(points_.size(), 0.0);
  for (size_t i = 1; i < points_.size(); ++i) {
    const double ds = std::hypot(
      points_[i].x - points_[i - 1].x, points_[i].y - points_[i - 1].y);
    cum_arc_[i] = cum_arc_[i - 1] + ds;
  }
  current_index_ = 0;
  progress_ = 0.0;
  state_ = TrajState::kReady;
  return true;
}

bool TrajectoryManager::validate(const std::vector<TrajPoint> & points)
{
  if (points.size() < limits_.min_points) {
    last_error_ = "轨迹点数不足";
    return false;
  }
  for (size_t i = 0; i < points.size(); ++i) {
    if (!(points[i].v_ref > 0.0)) {
      std::ostringstream oss;
      oss << "点 " << i << " v_ref 必须为正";
      last_error_ = oss.str();
      return false;
    }
    if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y) ||
      !std::isfinite(points[i].yaw) || !std::isfinite(points[i].curvature))
    {
      std::ostringstream oss;
      oss << "点 " << i << " 含非法数值";
      last_error_ = oss.str();
      return false;
    }
    if (i > 0) {
      const double ds = std::hypot(
        points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
      if (ds > limits_.max_point_gap) {
        std::ostringstream oss;
        oss << "点 " << i - 1 << "→" << i << " 间距 " << ds << " 超过 "
            << limits_.max_point_gap;
        last_error_ = oss.str();
        return false;
      }
    }
  }
  return true;
}

bool TrajectoryManager::start()
{
  if (state_ == TrajState::kReady) {
    state_ = TrajState::kRunning;
    return true;
  }
  if (state_ == TrajState::kFinished) {
    // 重跑同一轨迹
    current_index_ = 0;
    progress_ = 0.0;
    state_ = TrajState::kRunning;
    return true;
  }
  last_error_ = std::string("START 非法:当前状态 ") + toString(state_);
  return false;
}

bool TrajectoryManager::pause()
{
  if (state_ == TrajState::kRunning) {
    state_ = TrajState::kPaused;
    return true;
  }
  last_error_ = std::string("PAUSE 非法:当前状态 ") + toString(state_);
  return false;
}

bool TrajectoryManager::resume()
{
  if (state_ == TrajState::kPaused) {
    // 断点恢复:current_index_ / progress_ 保持不变
    state_ = TrajState::kRunning;
    return true;
  }
  last_error_ = std::string("RESUME 非法:当前状态 ") + toString(state_);
  return false;
}

bool TrajectoryManager::stop()
{
  if (state_ == TrajState::kRunning || state_ == TrajState::kPaused ||
    state_ == TrajState::kFinished)
  {
    current_index_ = 0;
    progress_ = 0.0;
    state_ = TrajState::kReady;
    return true;
  }
  last_error_ = std::string("STOP 非法:当前状态 ") + toString(state_);
  return false;
}

bool TrajectoryManager::reset()
{
  if (state_ == TrajState::kFault) {
    points_.clear();
    cum_arc_.clear();
    traj_id_ = 0;
    current_index_ = 0;
    progress_ = 0.0;
    state_ = TrajState::kIdle;
    return true;
  }
  last_error_ = std::string("RESET 非法:当前状态 ") + toString(state_);
  return false;
}

void TrajectoryManager::fault()
{
  state_ = TrajState::kFault;
}

TrackingRef TrajectoryManager::update(double x, double y, double yaw)
{
  TrackingRef ref;
  if (state_ != TrajState::kRunning || points_.empty()) {
    return ref;
  }

  // 最近点搜索:从当前索引向前搜索(带少量回退容忍),避免全局搜索跳点
  const size_t search_back = 5;
  const size_t start_idx = current_index_ > search_back ?
    current_index_ - search_back : 0;
  size_t best = start_idx;
  double best_d2 = 1e18;
  for (size_t i = start_idx; i < points_.size(); ++i) {
    const double dx = points_[i].x - x;
    const double dy = points_[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
    // 距离开始单调增大且已远离时提前终止(局部搜索)
    if (i > best + 50) {
      break;
    }
  }
  current_index_ = best;
  const TrajPoint & p = points_[best];

  // 误差求解:e_y 为位置误差在参考航向左法向上的投影;e_θ 为航向差
  const double dx = x - p.x;
  const double dy = y - p.y;
  ref.lateral_error = -std::sin(p.yaw) * dx + std::cos(p.yaw) * dy;
  ref.heading_error = wrapAngle(yaw - p.yaw);
  ref.point = p;

  // 进度:按累计弧长
  const double total = cum_arc_.back();
  progress_ = total > 1e-9 ? 100.0 * cum_arc_[best] / total : 100.0;
  ref.progress = progress_;

  // 到点判定:最近点为末点且距离小于容差
  const double dist_to_goal = std::hypot(
    points_.back().x - x, points_.back().y - y);
  if (best + 1 >= points_.size() && dist_to_goal < limits_.goal_tolerance) {
    ref.goal_reached = true;
    progress_ = 100.0;
    ref.progress = 100.0;
    state_ = TrajState::kFinished;
  }
  return ref;
}

}  // namespace tower_nav
