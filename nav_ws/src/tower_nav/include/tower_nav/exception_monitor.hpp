// 异常监测核心(纯逻辑,可单测):
// 话题心跳超时、定位失效、横向偏差超阈、急停、任务超时的集中监测与故障码管理。
#pragma once

#include <cstdint>
#include <string>

namespace tower_nav
{

/// 故障码(与 tower_nav_msgs/Fault 常量一致)
enum class FaultCode : uint16_t
{
  kNone = 0,
  kConfigInvalid = 1,
  kImuTimeout = 2,
  kEncoderTimeout = 3,
  kPoseTimeout = 4,
  kLocInvalid = 5,
  kLateralDev = 6,
  kEstop = 7,
  kMissionTimeout = 8,
  kTrajInvalid = 9,
};

const char * describe(FaultCode code);

/// 异常监测参数
struct ExceptionParams
{
  double imu_timeout{0.5};        // s
  double encoder_timeout{0.5};    // s
  double pose_timeout{0.5};       // s
  double lateral_dev_threshold{0.30};   // m
  double mission_timeout{1200.0};  // s
  double lateral_dev_hold{0.3};    // 偏差需持续超阈时长 s(抗瞬时噪声)
};

/// 异常监测器:馈入各信号时间戳与数值,check() 返回当前最高优先级故障
class ExceptionMonitor
{
public:
  explicit ExceptionMonitor(const ExceptionParams & params);

  // ---- 信号馈入 ----
  void feedImu(double stamp) {last_imu_ = stamp;}
  void feedEncoder(double stamp) {last_encoder_ = stamp;}
  void feedPose(double stamp) {last_pose_ = stamp;}
  void feedLocValid(bool valid) {loc_valid_ = valid;}
  void feedLateralError(double stamp, double e_y);
  void feedEstop(bool estop) {estop_ = estop;}

  /// 任务开始/结束(任务超时监测)
  void missionStarted(double stamp) {mission_start_ = stamp; mission_active_ = true;}
  void missionEnded() {mission_active_ = false;}

  /// 跟踪激活标志(仅 EXECUTING 时监测横向偏差)
  void setTrackingActive(bool active);

  /// 周期检查:返回当前故障码(kNone 表示正常)。
  /// 故障一旦触发即锁存,直到 reset()。
  FaultCode check(double now);

  /// 复位:清除锁存故障。急停仍激活时复位失败返回 false。
  bool reset();

  FaultCode activeFault() const {return latched_;}

private:
  ExceptionParams params_;
  double last_imu_{-1.0};
  double last_encoder_{-1.0};
  double last_pose_{-1.0};
  bool loc_valid_{true};
  bool estop_{false};
  bool tracking_active_{false};
  double lateral_exceed_since_{-1.0};
  double mission_start_{-1.0};
  bool mission_active_{false};
  FaultCode latched_{FaultCode::kNone};
};

}  // namespace tower_nav
