// 导航控制核心实现。
#include "tower_nav/tracking_controller.hpp"

#include <algorithm>
#include <cmath>

namespace tower_nav
{

TrackingController::TrackingController(const ControlParams & params)
: params_(params)
{
}

double TrackingController::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

void TrackingController::forceStop()
{
  last_ = ControlOutput{};
}

ControlOutput TrackingController::update(
  double v_ref, double curvature, double e_y, double e_theta,
  double dt, bool enabled)
{
  double v_des = 0.0;
  double w_des = 0.0;

  if (enabled) {
    v_des = v_ref;
    // 前馈 + 反馈:ω = v·κ + k_y·e_y + k_θ·e_θ
    // 误差定义:e_y 左偏为正 → 需向右转(负 ω);故反馈取负号
    const double w_ff = v_des * curvature;
    const double w_fb = -(params_.k_y * e_y + params_.k_theta * e_theta);
    w_des = w_ff + w_fb;
  }

  // 速度限幅
  v_des = clamp(v_des, -params_.max_linear_vel, params_.max_linear_vel);
  w_des = clamp(w_des, -params_.max_angular_vel, params_.max_angular_vel);

  // 加速度限幅(相对上一拍)
  if (dt > 0.0) {
    const double dv_max = params_.max_linear_acc * dt;
    const double dw_max = params_.max_angular_acc * dt;
    v_des = clamp(v_des, last_.v - dv_max, last_.v + dv_max);
    w_des = clamp(w_des, last_.omega - dw_max, last_.omega + dw_max);
  }

  // 轮速限幅:超限时等比例缩减 v 与 ω
  const double vl = (v_des - 0.5 * w_des * params_.wheel_separation) / params_.wheel_radius;
  const double vr = (v_des + 0.5 * w_des * params_.wheel_separation) / params_.wheel_radius;
  const double max_wheel = std::max(std::abs(vl), std::abs(vr));
  if (max_wheel > params_.max_wheel_vel && max_wheel > 1e-9) {
    const double scale = params_.max_wheel_vel / max_wheel;
    v_des *= scale;
    w_des *= scale;
  }

  last_ = ControlOutput{v_des, w_des};
  return last_;
}

}  // namespace tower_nav
