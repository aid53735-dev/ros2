// TrajectoryManager 单元测试:状态机迁移全覆盖、最近点/误差/进度/到点判定。
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "tower_nav/trajectory_manager.hpp"

namespace
{

using tower_nav::TrajectoryManager;
using tower_nav::TrajPoint;
using tower_nav::TrajState;

TrajectoryManager::Limits defaultLimits()
{
  TrajectoryManager::Limits lim;
  lim.max_point_gap = 1.0;
  lim.goal_tolerance = 0.15;
  lim.min_points = 2;
  return lim;
}

// 直线轨迹:沿 +x,每 0.1 m 一点,长度 length
std::vector<TrajPoint> lineTraj(double length, double v = 0.15)
{
  std::vector<TrajPoint> pts;
  for (double x = 0.0; x <= length + 1e-9; x += 0.1) {
    pts.push_back({x, 0.0, 0.0, v, 0.0});
  }
  return pts;
}

TEST(TrajectoryManager, InitialStateIdle)
{
  TrajectoryManager m(defaultLimits());
  EXPECT_EQ(m.state(), TrajState::kIdle);
  EXPECT_EQ(m.trajId(), 0);
}

TEST(TrajectoryManager, LoadValidTrajectory)
{
  TrajectoryManager m(defaultLimits());
  EXPECT_TRUE(m.loadPoints(1, lineTraj(2.0)));
  EXPECT_EQ(m.state(), TrajState::kReady);
  EXPECT_EQ(m.trajId(), 1);
}

TEST(TrajectoryManager, RejectTooFewPoints)
{
  TrajectoryManager m(defaultLimits());
  EXPECT_FALSE(m.loadPoints(1, {{0, 0, 0, 0.1, 0}}));
  EXPECT_EQ(m.state(), TrajState::kIdle);
}

TEST(TrajectoryManager, RejectNonPositiveVref)
{
  TrajectoryManager m(defaultLimits());
  std::vector<TrajPoint> pts = {{0, 0, 0, 0.1, 0}, {0.1, 0, 0, 0.0, 0}};
  EXPECT_FALSE(m.loadPoints(1, pts));
}

TEST(TrajectoryManager, RejectLargeGap)
{
  TrajectoryManager m(defaultLimits());
  std::vector<TrajPoint> pts = {{0, 0, 0, 0.1, 0}, {5.0, 0, 0, 0.1, 0}};
  EXPECT_FALSE(m.loadPoints(1, pts));
}

TEST(TrajectoryManager, RejectNonFinite)
{
  TrajectoryManager m(defaultLimits());
  std::vector<TrajPoint> pts = {{0, 0, 0, 0.1, 0},
    {std::nan(""), 0, 0, 0.1, 0}};
  EXPECT_FALSE(m.loadPoints(1, pts));
}

TEST(TrajectoryManager, StateMachineFullTransitions)
{
  TrajectoryManager m(defaultLimits());
  // IDLE:除加载外全部非法
  EXPECT_FALSE(m.start());
  EXPECT_FALSE(m.pause());
  EXPECT_FALSE(m.resume());
  EXPECT_FALSE(m.stop());
  EXPECT_FALSE(m.reset());

  ASSERT_TRUE(m.loadPoints(2, lineTraj(2.0)));
  // READY
  EXPECT_FALSE(m.pause());
  EXPECT_FALSE(m.resume());
  EXPECT_TRUE(m.start());
  EXPECT_EQ(m.state(), TrajState::kRunning);
  // RUNNING
  EXPECT_FALSE(m.start());
  EXPECT_FALSE(m.resume());
  EXPECT_TRUE(m.pause());
  EXPECT_EQ(m.state(), TrajState::kPaused);
  // PAUSED
  EXPECT_FALSE(m.start());
  EXPECT_FALSE(m.pause());
  EXPECT_TRUE(m.resume());
  EXPECT_EQ(m.state(), TrajState::kRunning);
  EXPECT_TRUE(m.pause());
  EXPECT_TRUE(m.stop());
  EXPECT_EQ(m.state(), TrajState::kReady);
  // FAULT
  m.fault();
  EXPECT_EQ(m.state(), TrajState::kFault);
  EXPECT_FALSE(m.start());
  EXPECT_FALSE(m.stop());
  EXPECT_TRUE(m.reset());
  EXPECT_EQ(m.state(), TrajState::kIdle);
  EXPECT_EQ(m.trajId(), 0);
}

TEST(TrajectoryManager, SwitchOnlyWhenNotRunning)
{
  TrajectoryManager m(defaultLimits());
  ASSERT_TRUE(m.loadPoints(1, lineTraj(2.0)));
  ASSERT_TRUE(m.start());
  // RUNNING 态禁止切换轨迹
  EXPECT_FALSE(m.loadPoints(2, lineTraj(3.0)));
  EXPECT_EQ(m.trajId(), 1);
  ASSERT_TRUE(m.pause());
  // PAUSED 态同样禁止
  EXPECT_FALSE(m.loadPoints(2, lineTraj(3.0)));
  ASSERT_TRUE(m.stop());
  // READY 态允许
  EXPECT_TRUE(m.loadPoints(2, lineTraj(3.0)));
  EXPECT_EQ(m.trajId(), 2);
}

TEST(TrajectoryManager, LateralAndHeadingError)
{
  TrajectoryManager m(defaultLimits());
  ASSERT_TRUE(m.loadPoints(1, lineTraj(2.0)));
  ASSERT_TRUE(m.start());
  // 机器人在 (0.5, 0.2) 朝 0.1 rad:最近点 (0.5,0),e_y=+0.2(左侧),e_θ=0.1
  const auto ref = m.update(0.5, 0.2, 0.1);
  EXPECT_NEAR(ref.lateral_error, 0.2, 1e-9);
  EXPECT_NEAR(ref.heading_error, 0.1, 1e-9);
  EXPECT_NEAR(ref.point.x, 0.5, 1e-9);
  // 右侧为负
  const auto ref2 = m.update(0.5, -0.3, 0.0);
  EXPECT_NEAR(ref2.lateral_error, -0.3, 1e-9);
}

TEST(TrajectoryManager, ProgressMonotonic)
{
  TrajectoryManager m(defaultLimits());
  ASSERT_TRUE(m.loadPoints(1, lineTraj(2.0)));
  ASSERT_TRUE(m.start());
  const auto r1 = m.update(0.0, 0.0, 0.0);
  const auto r2 = m.update(1.0, 0.0, 0.0);
  const auto r3 = m.update(1.9, 0.0, 0.0);
  EXPECT_LT(r1.progress, r2.progress);
  EXPECT_LT(r2.progress, r3.progress);
  EXPECT_NEAR(r2.progress, 50.0, 3.0);
}

TEST(TrajectoryManager, GoalReachedFinishes)
{
  TrajectoryManager m(defaultLimits());
  ASSERT_TRUE(m.loadPoints(1, lineTraj(2.0)));
  ASSERT_TRUE(m.start());
  m.update(1.5, 0.0, 0.0);
  const auto ref = m.update(1.98, 0.0, 0.0);
  EXPECT_TRUE(ref.goal_reached);
  EXPECT_EQ(m.state(), TrajState::kFinished);
  EXPECT_DOUBLE_EQ(m.progress(), 100.0);
  // FINISHED 后可重新 START(进度清零)
  EXPECT_TRUE(m.start());
  EXPECT_EQ(m.state(), TrajState::kRunning);
  EXPECT_EQ(m.currentIndex(), 0u);
}

TEST(TrajectoryManager, PauseResumeKeepsProgress)
{
  TrajectoryManager m(defaultLimits());
  ASSERT_TRUE(m.loadPoints(1, lineTraj(2.0)));
  ASSERT_TRUE(m.start());
  m.update(1.0, 0.0, 0.0);
  const size_t idx = m.currentIndex();
  const double prog = m.progress();
  ASSERT_TRUE(m.pause());
  ASSERT_TRUE(m.resume());
  // 断点恢复:索引与进度不变
  EXPECT_EQ(m.currentIndex(), idx);
  EXPECT_DOUBLE_EQ(m.progress(), prog);
}

TEST(TrajectoryManager, StopClearsProgress)
{
  TrajectoryManager m(defaultLimits());
  ASSERT_TRUE(m.loadPoints(1, lineTraj(2.0)));
  ASSERT_TRUE(m.start());
  m.update(1.0, 0.0, 0.0);
  ASSERT_TRUE(m.stop());
  EXPECT_EQ(m.currentIndex(), 0u);
  EXPECT_DOUBLE_EQ(m.progress(), 0.0);
}

TEST(TrajectoryManager, UpdateInactiveReturnsZero)
{
  TrajectoryManager m(defaultLimits());
  ASSERT_TRUE(m.loadPoints(1, lineTraj(2.0)));
  // READY(未 START)时 update 返回零参考
  const auto ref = m.update(1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(ref.lateral_error, 0.0);
  EXPECT_FALSE(ref.goal_reached);
}

TEST(TrajectoryManager, LoadFromMissingFileFails)
{
  TrajectoryManager m(defaultLimits());
  EXPECT_FALSE(m.loadFromFile(1, "/nonexistent/traj.yaml"));
  EXPECT_FALSE(m.lastError().empty());
}

}  // namespace
