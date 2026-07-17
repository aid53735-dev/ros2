// DiffDriveSim 单元测试:运动学积分(直线/原地转/圆弧)、打滑与噪声注入。
#include <cmath>

#include <gtest/gtest.h>

#include "tower_nav_sim/diff_drive_sim.hpp"

namespace
{

using tower_nav_sim::DiffDriveSim;
using tower_nav_sim::SimParams;

SimParams cleanParams()
{
  SimParams p;
  p.wheel_separation = 0.40;
  p.wheel_radius = 0.08;
  // 无噪声,便于确定性验证
  p.imu_gyro_noise_std = 0.0;
  p.imu_gyro_bias = 0.0;
  p.imu_yaw_noise_std = 0.0;
  p.encoder_noise_std = 0.0;
  return p;
}

void run(DiffDriveSim & sim, double duration, double dt = 0.01)
{
  const int steps = static_cast<int>(std::round(duration / dt));
  for (int i = 0; i < steps; ++i) {
    sim.step(dt);
  }
}

TEST(DiffDriveKinematics, ForwardInverseConsistency)
{
  double v = 0.0, w = 0.0, wl = 0.0, wr = 0.0;
  tower_nav_sim::diffDriveInverse(0.2, 0.5, 0.08, 0.40, wl, wr);
  tower_nav_sim::diffDriveForward(wl, wr, 0.08, 0.40, v, w);
  EXPECT_NEAR(v, 0.2, 1e-12);
  EXPECT_NEAR(w, 0.5, 1e-12);
}

TEST(DiffDriveSim, StraightLine)
{
  DiffDriveSim sim(cleanParams());
  sim.setCommand(0.2, 0.0);
  run(sim, 10.0);
  EXPECT_NEAR(sim.state().x, 2.0, 1e-9);
  EXPECT_NEAR(sim.state().y, 0.0, 1e-9);
  EXPECT_NEAR(sim.state().yaw, 0.0, 1e-9);
  // 轮角 = 距离/半径
  EXPECT_NEAR(sim.state().left_wheel_angle, 2.0 / 0.08, 1e-6);
  EXPECT_NEAR(sim.state().right_wheel_angle, 2.0 / 0.08, 1e-6);
}

TEST(DiffDriveSim, SpinInPlace)
{
  DiffDriveSim sim(cleanParams());
  sim.setCommand(0.0, 0.5);
  run(sim, 4.0);
  EXPECT_NEAR(sim.state().yaw, 2.0, 1e-9);
  EXPECT_NEAR(sim.state().x, 0.0, 1e-9);
  EXPECT_NEAR(sim.state().y, 0.0, 1e-9);
  // 左右轮反向
  EXPECT_LT(sim.state().left_wheel_angle, 0.0);
  EXPECT_GT(sim.state().right_wheel_angle, 0.0);
}

TEST(DiffDriveSim, CircularArc)
{
  // v=0.2, w=0.2 → 半径 1 m;跑整圆周期 T=2π/0.2
  DiffDriveSim sim(cleanParams());
  sim.setCommand(0.2, 0.2);
  const double period = 2.0 * M_PI / 0.2;
  run(sim, period, 0.001);
  // 回到原点附近(中点法二阶精度;步数取整引入 ~dt/2 时间量化误差)
  EXPECT_NEAR(sim.state().x, 0.0, 1e-3);
  EXPECT_NEAR(sim.state().y, 0.0, 1e-3);
  EXPECT_NEAR(std::fmod(sim.state().yaw, 2.0 * M_PI), 0.0, 1e-4);
}

TEST(DiffDriveSim, InitialPose)
{
  DiffDriveSim sim(cleanParams());
  sim.setPose(3.0, 20.0, M_PI / 2.0);
  sim.setCommand(0.1, 0.0);
  run(sim, 5.0);
  // 沿 +y 方向前进
  EXPECT_NEAR(sim.state().x, 3.0, 1e-9);
  EXPECT_NEAR(sim.state().y, 20.5, 1e-9);
}

TEST(DiffDriveSim, SlipMakesEncoderOverCount)
{
  DiffDriveSim sim(cleanParams());
  sim.setSlipRatio(0.3);
  sim.setCommand(0.2, 0.0);
  run(sim, 10.0);
  const auto enc = sim.sampleEncoder();
  // 实际位移 2 m,但编码器按 1.3 倍计数
  EXPECT_NEAR(sim.state().x, 2.0, 1e-9);
  EXPECT_NEAR(enc.left_angle, 1.3 * 2.0 / 0.08, 1e-6);
  EXPECT_NEAR(enc.right_velocity, 1.3 * 0.2 / 0.08, 1e-9);
}

TEST(DiffDriveSim, CleanSensorsMatchTruth)
{
  DiffDriveSim sim(cleanParams());
  sim.setCommand(0.15, 0.1);
  run(sim, 3.0);
  const auto imu = sim.sampleImu();
  EXPECT_NEAR(imu.yaw, sim.state().yaw, 1e-12);
  EXPECT_NEAR(imu.gyro_z, sim.state().omega, 1e-12);
  const auto enc = sim.sampleEncoder();
  EXPECT_NEAR(enc.left_velocity, sim.state().left_wheel_velocity, 1e-12);
}

TEST(DiffDriveSim, GyroBiasApplied)
{
  SimParams p = cleanParams();
  p.imu_gyro_bias = 0.01;
  DiffDriveSim sim(p);
  sim.setCommand(0.0, 0.0);
  sim.step(0.01);
  const auto imu = sim.sampleImu();
  EXPECT_NEAR(imu.gyro_z, 0.01, 1e-12);
}

TEST(DiffDriveSim, NoiseIsReproducibleWithSeed)
{
  SimParams p = cleanParams();
  p.imu_yaw_noise_std = 0.01;
  p.noise_seed = 7;
  DiffDriveSim a(p), b(p);
  a.step(0.01);
  b.step(0.01);
  EXPECT_DOUBLE_EQ(a.sampleImu().yaw, b.sampleImu().yaw);
}

}  // namespace
