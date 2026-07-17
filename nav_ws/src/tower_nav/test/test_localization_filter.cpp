// LocalizationFilter 单元测试:融合数学、超时与发散检测。
#include <cmath>

#include <gtest/gtest.h>

#include "tower_nav/localization_filter.hpp"

namespace
{

using tower_nav::FusionParams;
using tower_nav::LocalizationFilter;

FusionParams defaultParams()
{
  FusionParams p;
  p.heading_alpha = 0.98;
  p.sensor_timeout = 0.5;
  p.cov_diverge_threshold = 5.0;
  return p;
}

// 以 100 Hz 馈入一致的 odom+imu 并推进
void runConsistent(
  LocalizationFilter & f, double t0, double duration,
  double v, double omega, double yaw0)
{
  const double dt = 0.01;
  double yaw = yaw0;
  for (double t = t0; t < t0 + duration; t += dt) {
    f.feedOdomVelocity(t, v, omega);
    f.feedImuHeading(t, yaw, omega);
    f.predict(t);
    yaw += omega * dt;
  }
}

TEST(LocalizationFilter, StraightLineIntegration)
{
  LocalizationFilter f(defaultParams());
  f.reset(0.0, 0.0, 0.0);
  runConsistent(f, 0.0, 10.0, 0.2, 0.0, 0.0);
  // 10 s × 0.2 m/s = 2 m,误差 < 2%
  EXPECT_NEAR(f.state().x, 2.0, 0.04);
  EXPECT_NEAR(f.state().y, 0.0, 0.01);
  EXPECT_TRUE(f.valid());
}

TEST(LocalizationFilter, HeadingFollowsImu)
{
  // 编码器声称 omega=0,IMU 报告航向持续变化 → 航向应跟随 IMU
  LocalizationFilter f(defaultParams());
  f.reset(0.0, 0.0, 0.0);
  const double dt = 0.01;
  double imu_yaw = 0.0;
  for (double t = 0.0; t < 5.0; t += dt) {
    imu_yaw = 0.1 * t;
    f.feedOdomVelocity(t, 0.0, 0.0);
    f.feedImuHeading(t, imu_yaw, 0.1);
    f.predict(t);
  }
  EXPECT_NEAR(f.state().yaw, 0.5, 0.02);
}

TEST(LocalizationFilter, ImuCorrectsEncoderDrift)
{
  // 编码器角速度有恒定偏差(如打滑),IMU 航向正确 → 融合航向靠近 IMU
  LocalizationFilter f(defaultParams());
  f.reset(0.0, 0.0, 0.0);
  const double dt = 0.01;
  for (double t = 0.0; t < 10.0; t += dt) {
    f.feedOdomVelocity(t, 0.0, 0.05);  // 假角速度:纯漂移
    f.feedImuHeading(t, 0.0, 0.0);     // IMU:真实航向恒 0
    f.predict(t);
  }
  // 无融合时漂移 0.5 rad;融合后应被抑制到很小
  EXPECT_LT(std::abs(f.state().yaw), 0.01);
}

TEST(LocalizationFilter, AngleWrapAtPi)
{
  // 航向跨 ±π 边界时融合不跳变
  LocalizationFilter f(defaultParams());
  f.reset(0.0, 0.0, M_PI - 0.01);
  runConsistent(f, 0.0, 1.0, 0.0, 0.1, M_PI - 0.01);
  // 期望绕过 π:π-0.01+0.1 ≈ -π+0.09
  EXPECT_NEAR(std::abs(f.state().yaw), M_PI - 0.09, 0.02);
}

TEST(LocalizationFilter, OdomTimeoutInvalidates)
{
  LocalizationFilter f(defaultParams());
  f.reset(0.0, 0.0, 0.0);
  runConsistent(f, 0.0, 1.0, 0.1, 0.0, 0.0);
  EXPECT_TRUE(f.valid());
  // 停止馈入 odom,仅推进时间与 IMU
  for (double t = 1.0; t < 2.0; t += 0.01) {
    f.feedImuHeading(t, 0.0, 0.0);
    f.predict(t);
  }
  EXPECT_FALSE(f.valid());
  EXPECT_TRUE(f.odomTimeout());
  EXPECT_FALSE(f.imuTimeout());
}

TEST(LocalizationFilter, ImuTimeoutInvalidates)
{
  LocalizationFilter f(defaultParams());
  f.reset(0.0, 0.0, 0.0);
  runConsistent(f, 0.0, 1.0, 0.1, 0.0, 0.0);
  EXPECT_TRUE(f.valid());
  for (double t = 1.0; t < 2.0; t += 0.01) {
    f.feedOdomVelocity(t, 0.1, 0.0);
    f.predict(t);
  }
  EXPECT_FALSE(f.valid());
  EXPECT_TRUE(f.imuTimeout());
  EXPECT_FALSE(f.odomTimeout());
}

TEST(LocalizationFilter, CovarianceGrowsWithTravel)
{
  LocalizationFilter f(defaultParams());
  f.reset(0.0, 0.0, 0.0);
  runConsistent(f, 0.0, 1.0, 0.2, 0.0, 0.0);
  const double cov_early = f.state().cov_xx;
  runConsistent(f, 1.0, 9.0, 0.2, 0.0, 0.0);
  EXPECT_GT(f.state().cov_xx, cov_early);
}

TEST(LocalizationFilter, CovarianceDivergenceInvalidates)
{
  FusionParams p = defaultParams();
  p.cov_diverge_threshold = 1e-5;  // 极低阈值,快速触发
  LocalizationFilter f(p);
  f.reset(0.0, 0.0, 0.0);
  runConsistent(f, 0.0, 5.0, 0.2, 0.0, 0.0);
  EXPECT_TRUE(f.covDiverged());
  EXPECT_FALSE(f.valid());
}

TEST(LocalizationFilter, ResetRestoresValidity)
{
  LocalizationFilter f(defaultParams());
  f.reset(0.0, 0.0, 0.0);
  // 触发超时
  f.predict(0.0);
  f.predict(1.0);
  EXPECT_FALSE(f.valid());
  // 复位后重新馈入
  f.reset(1.0, 2.0, 0.5);
  EXPECT_DOUBLE_EQ(f.state().x, 1.0);
  EXPECT_DOUBLE_EQ(f.state().yaw, 0.5);
  runConsistent(f, 2.0, 1.0, 0.1, 0.0, 0.5);
  EXPECT_TRUE(f.valid());
}

TEST(LocalizationFilter, NoPredictBeforeReset)
{
  LocalizationFilter f(defaultParams());
  f.predict(1.0);  // 未初始化时应无副作用
  EXPECT_FALSE(f.valid());
}

}  // namespace
