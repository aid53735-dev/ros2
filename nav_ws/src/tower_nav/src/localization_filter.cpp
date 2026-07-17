// 定位融合核心实现。
#include "tower_nav/localization_filter.hpp"

#include <algorithm>
#include <cmath>

namespace tower_nav
{

LocalizationFilter::LocalizationFilter(const FusionParams & params)
: params_(params)
{
}

double LocalizationFilter::wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

void LocalizationFilter::reset(double x, double y, double yaw)
{
  state_ = FusedState{};
  state_.x = x;
  state_.y = y;
  state_.yaw = yaw;
  state_.cov_xx = 1e-6;
  state_.cov_yy = 1e-6;
  state_.cov_yawyaw = 1e-6;
  state_.valid = true;
  initialized_ = true;
  last_predict_time_ = -1.0;
  last_odom_time_ = -1.0;
  last_imu_time_ = -1.0;
  imu_yaw_.reset();
  odom_timeout_ = false;
  imu_timeout_ = false;
  cov_diverged_ = false;
}

void LocalizationFilter::feedOdomVelocity(double stamp, double v, double omega)
{
  state_.v = v;
  state_.omega = omega;
  last_odom_time_ = stamp;
}

void LocalizationFilter::feedImuHeading(double stamp, double yaw, double yaw_rate)
{
  imu_yaw_ = yaw;
  imu_yaw_rate_ = yaw_rate;
  last_imu_time_ = stamp;
}

void LocalizationFilter::predict(double now)
{
  if (!initialized_) {
    return;
  }
  if (last_predict_time_ < 0.0) {
    last_predict_time_ = now;
    return;
  }
  const double dt = now - last_predict_time_;
  last_predict_time_ = now;
  if (dt <= 0.0) {
    return;
  }

  // ---- 航向融合:互补滤波 ----
  // 编码器角速度积分作为高频预测,IMU 航向作为低频校正基准。
  // yaw_pred = yaw + omega_enc*dt;yaw = alpha*(IMU 校正) + (1-alpha)*yaw_pred
  const double yaw_pred = state_.yaw + state_.omega * dt;
  if (imu_yaw_.has_value()) {
    const double innovation = wrapAngle(*imu_yaw_ - yaw_pred);
    state_.yaw = wrapAngle(yaw_pred + params_.heading_alpha * innovation);
  } else {
    state_.yaw = wrapAngle(yaw_pred);
  }

  // ---- 位置:里程计积分(中点航向)----
  const double yaw_mid = state_.yaw - 0.5 * state_.omega * dt;
  state_.x += state_.v * std::cos(yaw_mid) * dt;
  state_.y += state_.v * std::sin(yaw_mid) * dt;

  // ---- 协方差传播(简化对角模型)----
  const double speed_factor = 1.0 + std::abs(state_.v);
  state_.cov_xx += params_.pos_noise_rate * speed_factor * dt;
  state_.cov_yy += params_.pos_noise_rate * speed_factor * dt;
  if (imu_yaw_.has_value()) {
    // 有 IMU 校正时航向协方差收敛到测量水平
    state_.cov_yawyaw = std::max(
      1e-6, state_.cov_yawyaw * (1.0 - params_.heading_alpha) +
      params_.yaw_noise_rate * dt);
  } else {
    state_.cov_yawyaw += params_.yaw_noise_rate * dt * 100.0;  // 无校正,漂移加速
  }

  // ---- 有效性检查 ----
  odom_timeout_ = (last_odom_time_ < 0.0) ||
    (now - last_odom_time_ > params_.sensor_timeout);
  imu_timeout_ = (last_imu_time_ < 0.0) ||
    (now - last_imu_time_ > params_.sensor_timeout);
  cov_diverged_ = (state_.cov_xx > params_.cov_diverge_threshold) ||
    (state_.cov_yy > params_.cov_diverge_threshold);
  state_.valid = !odom_timeout_ && !imu_timeout_ && !cov_diverged_;
}

}  // namespace tower_nav
