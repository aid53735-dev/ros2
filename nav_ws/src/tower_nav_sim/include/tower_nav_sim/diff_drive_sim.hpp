// 双轮差速运动学仿真核心(纯逻辑,不依赖 ROS 运行时,便于单元测试)。
// 按 /cmd_vel 指令积分位姿,生成轮角(编码器)与 IMU 真值;
// 支持 IMU 噪声/零偏配置与打滑扰动注入。
#pragma once

#include <cstdint>
#include <random>

namespace tower_nav_sim
{

/// 仿真参数
struct SimParams
{
  double wheel_separation{0.40};   // 轮距 m
  double wheel_radius{0.08};       // 轮半径 m
  double imu_gyro_noise_std{0.0};  // 陀螺角速度噪声标准差 rad/s
  double imu_gyro_bias{0.0};       // 陀螺零偏 rad/s
  double imu_yaw_noise_std{0.0};   // 姿态 yaw 噪声标准差 rad
  double encoder_noise_std{0.0};   // 编码器轮速噪声标准差 rad/s
  uint32_t noise_seed{42};         // 随机种子(可复现)
};

/// 仿真状态快照
struct SimState
{
  double x{0.0};        // 展开坐标 x(周向弧长)m
  double y{0.0};        // 展开坐标 y(高度)m
  double yaw{0.0};      // 航向 rad
  double v{0.0};        // 线速度 m/s
  double omega{0.0};    // 角速度 rad/s
  double left_wheel_angle{0.0};    // 左轮累计转角 rad(真值)
  double right_wheel_angle{0.0};   // 右轮累计转角 rad(真值)
  double left_wheel_velocity{0.0};   // 左轮角速度 rad/s(真值)
  double right_wheel_velocity{0.0};  // 右轮角速度 rad/s(真值)
};

/// IMU 采样输出(含噪声与零偏)
struct ImuSample
{
  double yaw{0.0};          // 姿态 yaw(含噪声)
  double gyro_z{0.0};       // z 轴角速度(含噪声与零偏)
};

/// 编码器采样输出(含噪声与打滑)
struct EncoderSample
{
  double left_angle{0.0};
  double right_angle{0.0};
  double left_velocity{0.0};
  double right_velocity{0.0};
};

/// 双轮差速仿真器
class DiffDriveSim
{
public:
  explicit DiffDriveSim(const SimParams & params);

  /// 设置速度指令(来自 /cmd_vel)
  void setCommand(double v, double omega);

  /// 前向积分 dt 秒(中点法,保证圆弧轨迹精度)
  void step(double dt);

  /// 打滑注入:编码器轮角按 slip_ratio 比例多计
  /// (轮子转了但机器人没动),0 = 无打滑
  void setSlipRatio(double slip_ratio) {slip_ratio_ = slip_ratio;}
  double slipRatio() const {return slip_ratio_;}

  /// 设置初始位姿
  void setPose(double x, double y, double yaw);

  const SimState & state() const {return state_;}
  const SimParams & params() const {return params_;}

  /// 采样 IMU(叠加噪声与零偏)
  ImuSample sampleImu();

  /// 采样编码器(叠加噪声与打滑)
  EncoderSample sampleEncoder();

private:
  SimParams params_;
  SimState state_;
  double cmd_v_{0.0};
  double cmd_omega_{0.0};
  double slip_ratio_{0.0};
  // 打滑影响下编码器侧的累计轮角(与真值分离)
  double enc_left_angle_{0.0};
  double enc_right_angle_{0.0};
  std::mt19937 rng_;
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

/// 差速运动学正解:由左右轮角速度求 (v, omega)
/// 供仿真与定位共用的公式参考实现
void diffDriveForward(
  double left_wheel_vel, double right_wheel_vel,
  double wheel_radius, double wheel_separation,
  double & v, double & omega);

/// 差速运动学逆解:由 (v, omega) 求左右轮角速度
void diffDriveInverse(
  double v, double omega,
  double wheel_radius, double wheel_separation,
  double & left_wheel_vel, double & right_wheel_vel);

}  // namespace tower_nav_sim
