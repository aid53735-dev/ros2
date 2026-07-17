// 双轮差速仿真核心实现。
#include "tower_nav_sim/diff_drive_sim.hpp"

#include <cmath>

namespace tower_nav_sim
{

void diffDriveForward(
  double left_wheel_vel, double right_wheel_vel,
  double wheel_radius, double wheel_separation,
  double & v, double & omega)
{
  const double vl = left_wheel_vel * wheel_radius;
  const double vr = right_wheel_vel * wheel_radius;
  v = 0.5 * (vl + vr);
  omega = (vr - vl) / wheel_separation;
}

void diffDriveInverse(
  double v, double omega,
  double wheel_radius, double wheel_separation,
  double & left_wheel_vel, double & right_wheel_vel)
{
  const double vl = v - 0.5 * omega * wheel_separation;
  const double vr = v + 0.5 * omega * wheel_separation;
  left_wheel_vel = vl / wheel_radius;
  right_wheel_vel = vr / wheel_radius;
}

DiffDriveSim::DiffDriveSim(const SimParams & params)
: params_(params), rng_(params.noise_seed)
{
}

void DiffDriveSim::setCommand(double v, double omega)
{
  cmd_v_ = v;
  cmd_omega_ = omega;
}

void DiffDriveSim::setPose(double x, double y, double yaw)
{
  state_.x = x;
  state_.y = y;
  state_.yaw = yaw;
}

void DiffDriveSim::step(double dt)
{
  // 速度立即跟随指令(执行器动力学不建模,加速度限幅在控制器侧)
  state_.v = cmd_v_;
  state_.omega = cmd_omega_;

  // 中点法积分:航向取步内中值,圆弧轨迹二阶精度
  const double yaw_mid = state_.yaw + 0.5 * state_.omega * dt;
  state_.x += state_.v * std::cos(yaw_mid) * dt;
  state_.y += state_.v * std::sin(yaw_mid) * dt;
  state_.yaw += state_.omega * dt;

  // 真值轮速与轮角
  diffDriveInverse(
    state_.v, state_.omega, params_.wheel_radius, params_.wheel_separation,
    state_.left_wheel_velocity, state_.right_wheel_velocity);
  state_.left_wheel_angle += state_.left_wheel_velocity * dt;
  state_.right_wheel_angle += state_.right_wheel_velocity * dt;

  // 编码器侧轮角:打滑时轮子空转,编码器多计 (1+slip) 倍
  const double slip_scale = 1.0 + slip_ratio_;
  enc_left_angle_ += state_.left_wheel_velocity * slip_scale * dt;
  enc_right_angle_ += state_.right_wheel_velocity * slip_scale * dt;
}

ImuSample DiffDriveSim::sampleImu()
{
  ImuSample s;
  s.yaw = state_.yaw + params_.imu_yaw_noise_std * gauss_(rng_);
  s.gyro_z = state_.omega + params_.imu_gyro_bias +
    params_.imu_gyro_noise_std * gauss_(rng_);
  return s;
}

EncoderSample DiffDriveSim::sampleEncoder()
{
  EncoderSample s;
  const double slip_scale = 1.0 + slip_ratio_;
  s.left_angle = enc_left_angle_ + params_.encoder_noise_std * gauss_(rng_);
  s.right_angle = enc_right_angle_ + params_.encoder_noise_std * gauss_(rng_);
  s.left_velocity = state_.left_wheel_velocity * slip_scale +
    params_.encoder_noise_std * gauss_(rng_);
  s.right_velocity = state_.right_wheel_velocity * slip_scale +
    params_.encoder_noise_std * gauss_(rng_);
  return s;
}

}  // namespace tower_nav_sim
