// TrackingController 单元测试:控制律数值、误差符号、限幅、禁用零速。
#include <cmath>

#include <gtest/gtest.h>

#include "tower_nav/tracking_controller.hpp"

namespace
{

using tower_nav::ControlParams;
using tower_nav::TrackingController;

ControlParams defaultParams()
{
  ControlParams p;
  p.k_y = 1.2;
  p.k_theta = 1.8;
  p.max_linear_vel = 0.30;
  p.max_angular_vel = 0.60;
  p.max_linear_acc = 0.30;
  p.max_angular_acc = 1.20;
  p.max_wheel_vel = 5.0;
  p.wheel_radius = 0.08;
  p.wheel_separation = 0.40;
  return p;
}

// 通过多拍逼近稳态(绕开加速度限幅影响)
tower_nav::ControlOutput steady(
  TrackingController & c, double v_ref, double kappa, double ey, double eth)
{
  tower_nav::ControlOutput out;
  for (int i = 0; i < 200; ++i) {
    out = c.update(v_ref, kappa, ey, eth, 0.02, true);
  }
  return out;
}

TEST(TrackingController, FeedforwardOnly)
{
  TrackingController c(defaultParams());
  // 无误差:ω = v·κ = 0.15 * 0.5
  const auto out = steady(c, 0.15, 0.5, 0.0, 0.0);
  EXPECT_NEAR(out.v, 0.15, 1e-9);
  EXPECT_NEAR(out.omega, 0.075, 1e-9);
}

TEST(TrackingController, FeedbackSigns)
{
  TrackingController c(defaultParams());
  // 左偏(e_y>0)→ 应右转:ω < ω_ff
  const auto left = steady(c, 0.15, 0.0, 0.1, 0.0);
  EXPECT_NEAR(left.omega, -1.2 * 0.1, 1e-9);
  // 右偏 → 左转
  TrackingController c2(defaultParams());
  const auto right = steady(c2, 0.15, 0.0, -0.1, 0.0);
  EXPECT_NEAR(right.omega, 1.2 * 0.1, 1e-9);
  // 航向偏左(e_θ>0)→ 右转
  TrackingController c3(defaultParams());
  const auto head = steady(c3, 0.15, 0.0, 0.0, 0.2);
  EXPECT_NEAR(head.omega, -1.8 * 0.2, 1e-9);
}

TEST(TrackingController, CombinedLaw)
{
  TrackingController c(defaultParams());
  // ω = v·κ - (k_y·e_y + k_θ·e_θ) = 0.15*0.4 - (1.2*0.05 + 1.8*(-0.1))
  const auto out = steady(c, 0.15, 0.4, 0.05, -0.1);
  EXPECT_NEAR(out.omega, 0.06 - (0.06 - 0.18), 1e-9);
}

TEST(TrackingController, LinearVelocitySaturation)
{
  TrackingController c(defaultParams());
  const auto out = steady(c, 1.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(out.v, 0.30, 1e-9);
}

TEST(TrackingController, AngularVelocitySaturation)
{
  TrackingController c(defaultParams());
  const auto out = steady(c, 0.1, 0.0, 1.0, 1.0);  // 大误差
  EXPECT_NEAR(out.omega, -0.60, 1e-9);
}

TEST(TrackingController, AccelerationLimit)
{
  TrackingController c(defaultParams());
  // 首拍:dt=0.02,dv_max = 0.30*0.02 = 0.006
  const auto out = c.update(0.3, 0.0, 0.0, 0.0, 0.02, true);
  EXPECT_NEAR(out.v, 0.006, 1e-9);
  const auto out2 = c.update(0.3, 0.0, 0.0, 0.0, 0.02, true);
  EXPECT_NEAR(out2.v, 0.012, 1e-9);
}

TEST(TrackingController, WheelVelocityLimitScales)
{
  ControlParams p = defaultParams();
  p.max_wheel_vel = 2.0;  // 收紧轮速限制:v=0.3 → 轮速 3.75 rad/s 超限
  p.max_linear_acc = 100.0;  // 放开加速度限幅便于观察
  p.max_angular_acc = 100.0;
  TrackingController c(p);
  const auto out = c.update(0.3, 0.0, 0.0, 0.0, 0.02, true);
  // 缩放到轮速恰为 2.0:v = 2.0*0.08 = 0.16
  EXPECT_NEAR(out.v, 0.16, 1e-9);
  // 等比例缩放不改变 v/ω 比(此处 ω=0)
  EXPECT_NEAR(out.omega, 0.0, 1e-12);
}

TEST(TrackingController, DisabledDeceleratesToZero)
{
  TrackingController c(defaultParams());
  steady(c, 0.15, 0.0, 0.0, 0.0);
  EXPECT_GT(c.lastOutput().v, 0.1);
  // 禁用后按减速度限幅降到零
  for (int i = 0; i < 100; ++i) {
    c.update(0.15, 0.0, 0.0, 0.0, 0.02, false);
  }
  EXPECT_NEAR(c.lastOutput().v, 0.0, 1e-9);
  EXPECT_NEAR(c.lastOutput().omega, 0.0, 1e-9);
}

TEST(TrackingController, ForceStopImmediate)
{
  TrackingController c(defaultParams());
  steady(c, 0.15, 0.0, 0.0, 0.0);
  c.forceStop();
  EXPECT_DOUBLE_EQ(c.lastOutput().v, 0.0);
  EXPECT_DOUBLE_EQ(c.lastOutput().omega, 0.0);
}

}  // namespace
