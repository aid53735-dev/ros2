// 导航控制核心(纯逻辑,可单测):
// 控制律:ω = ω_ff + ω_fb = v·κ + (k_y·e_y + k_θ·e_θ);
// 安全限幅:最大线速度/角速度/加速度/左右轮速;FAULT/急停强制零速。
#pragma once

namespace tower_nav
{

/// 控制参数
struct ControlParams
{
  double k_y{1.2};                // 横向误差增益
  double k_theta{1.8};            // 航向误差增益
  double max_linear_vel{0.30};    // 最大线速度 m/s
  double max_angular_vel{0.60};   // 最大角速度 rad/s
  double max_linear_acc{0.30};    // 最大线加速度 m/s^2
  double max_angular_acc{1.20};   // 最大角加速度 rad/s^2
  double max_wheel_vel{5.0};      // 轮角速度上限 rad/s
  double wheel_radius{0.08};
  double wheel_separation{0.40};
};

/// 控制输出
struct ControlOutput
{
  double v{0.0};
  double omega{0.0};
};

/// 轨迹跟踪控制器
class TrackingController
{
public:
  explicit TrackingController(const ControlParams & params);

  /// 计算一拍控制量。
  /// v_ref/curvature:参考速度与曲率;e_y/e_theta:横向/航向误差;
  /// dt:控制周期;enabled=false(FAULT/急停/非 RUNNING)时强制零速(仍受减速度限幅)。
  ControlOutput update(
    double v_ref, double curvature, double e_y, double e_theta,
    double dt, bool enabled);

  /// 立即强制零速(急停:跳过加速度限幅)
  void forceStop();

  const ControlOutput & lastOutput() const {return last_;}

private:
  static double clamp(double v, double lo, double hi);

  ControlParams params_;
  ControlOutput last_;
};

}  // namespace tower_nav
