// 定位融合核心(纯逻辑,可单测):
// 航向:IMU 为主、编码器角速度为辅的互补滤波抑制漂移;
// 位置:里程计积分;协方差传播;超时/发散检测。
#pragma once

#include <optional>

namespace tower_nav
{

/// 融合参数
struct FusionParams
{
  double heading_alpha{0.98};        // 互补滤波系数:IMU 航向权重
  double sensor_timeout{0.5};        // 传感器超时阈值 s
  double cov_diverge_threshold{5.0};  // 位置协方差发散阈值 m^2
  double pos_noise_rate{1e-4};       // 位置过程噪声速率 m^2/s(随速度放大)
  double yaw_noise_rate{1e-6};       // 航向过程噪声速率 rad^2/s
};

/// 融合输出状态
struct FusedState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double omega{0.0};
  double cov_xx{0.0};
  double cov_yy{0.0};
  double cov_yawyaw{0.0};
  bool valid{false};      // 定位有效标志
};

/// 互补滤波式定位融合器
class LocalizationFilter
{
public:
  explicit LocalizationFilter(const FusionParams & params);

  /// 设置初始位姿
  void reset(double x, double y, double yaw);

  /// 馈入编码器解算结果(时间戳 s)
  void feedOdomVelocity(double stamp, double v, double omega);

  /// 馈入 IMU 航向与角速度(时间戳 s)
  void feedImuHeading(double stamp, double yaw, double yaw_rate);

  /// 时间推进到 now:积分位置、传播协方差、检查超时/发散
  void predict(double now);

  const FusedState & state() const {return state_;}

  /// 定位是否有效(未超时且协方差未发散)
  bool valid() const {return state_.valid;}

  /// 诊断:各输入是否超时
  bool odomTimeout() const {return odom_timeout_;}
  bool imuTimeout() const {return imu_timeout_;}
  bool covDiverged() const {return cov_diverged_;}

private:
  static double wrapAngle(double a);

  FusionParams params_;
  FusedState state_;
  double last_predict_time_{-1.0};
  double last_odom_time_{-1.0};
  double last_imu_time_{-1.0};
  std::optional<double> imu_yaw_;
  double imu_yaw_rate_{0.0};
  bool initialized_{false};
  bool odom_timeout_{false};
  bool imu_timeout_{false};
  bool cov_diverged_{false};
};

}  // namespace tower_nav
