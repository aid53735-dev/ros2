// 异常监测核心实现。
#include "tower_nav/exception_monitor.hpp"

#include <cmath>

namespace tower_nav
{

const char * describe(FaultCode code)
{
  switch (code) {
    case FaultCode::kNone: return "正常";
    case FaultCode::kConfigInvalid: return "配置校验失败";
    case FaultCode::kImuTimeout: return "IMU 数据超时";
    case FaultCode::kEncoderTimeout: return "编码器数据超时";
    case FaultCode::kPoseTimeout: return "定位位姿超时";
    case FaultCode::kLocInvalid: return "定位失效";
    case FaultCode::kLateralDev: return "横向偏差超阈";
    case FaultCode::kEstop: return "急停触发";
    case FaultCode::kMissionTimeout: return "任务超时";
    case FaultCode::kTrajInvalid: return "轨迹校验失败";
  }
  return "未知故障";
}

ExceptionMonitor::ExceptionMonitor(const ExceptionParams & params)
: params_(params)
{
}

void ExceptionMonitor::feedLateralError(double stamp, double e_y)
{
  if (!tracking_active_) {
    lateral_exceed_since_ = -1.0;
    return;
  }
  if (std::abs(e_y) > params_.lateral_dev_threshold) {
    if (lateral_exceed_since_ < 0.0) {
      lateral_exceed_since_ = stamp;
    }
  } else {
    lateral_exceed_since_ = -1.0;
  }
}

void ExceptionMonitor::setTrackingActive(bool active)
{
  tracking_active_ = active;
  if (!active) {
    lateral_exceed_since_ = -1.0;
  }
}

FaultCode ExceptionMonitor::check(double now)
{
  // 急停最高优先级:抢占任何已锁存故障
  if (estop_) {
    latched_ = FaultCode::kEstop;
    return latched_;
  }
  if (latched_ != FaultCode::kNone) {
    return latched_;
  }

  // 优先级:传感器超时 > 定位失效 > 横向偏差 > 任务超时
  if (last_imu_ >= 0.0 && now - last_imu_ > params_.imu_timeout) {
    latched_ = FaultCode::kImuTimeout;
  } else if (last_encoder_ >= 0.0 && now - last_encoder_ > params_.encoder_timeout) {
    latched_ = FaultCode::kEncoderTimeout;
  } else if (last_pose_ >= 0.0 && now - last_pose_ > params_.pose_timeout) {
    latched_ = FaultCode::kPoseTimeout;
  } else if (!loc_valid_) {
    latched_ = FaultCode::kLocInvalid;
  } else if (lateral_exceed_since_ >= 0.0 &&
    now - lateral_exceed_since_ >= params_.lateral_dev_hold)
  {
    latched_ = FaultCode::kLateralDev;
  } else if (mission_active_ && mission_start_ >= 0.0 &&
    now - mission_start_ > params_.mission_timeout)
  {
    latched_ = FaultCode::kMissionTimeout;
  }
  return latched_;
}

bool ExceptionMonitor::reset()
{
  if (estop_) {
    return false;  // 急停未解除,禁止复位
  }
  latched_ = FaultCode::kNone;
  lateral_exceed_since_ = -1.0;
  // 超时时间戳清零:等待新数据重新建立心跳
  last_imu_ = -1.0;
  last_encoder_ = -1.0;
  last_pose_ = -1.0;
  loc_valid_ = true;
  return true;
}

}  // namespace tower_nav
