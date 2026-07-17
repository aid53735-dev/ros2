// ExceptionMonitor 单元测试:各触发条件、优先级、锁存与复位。
#include <gtest/gtest.h>

#include "tower_nav/exception_monitor.hpp"

namespace
{

using tower_nav::ExceptionMonitor;
using tower_nav::ExceptionParams;
using tower_nav::FaultCode;

ExceptionParams defaultParams()
{
  ExceptionParams p;
  p.imu_timeout = 0.5;
  p.encoder_timeout = 0.5;
  p.pose_timeout = 0.5;
  p.lateral_dev_threshold = 0.30;
  p.lateral_dev_hold = 0.3;
  p.mission_timeout = 100.0;
  return p;
}

// 馈入全部健康信号
void feedHealthy(ExceptionMonitor & m, double t)
{
  m.feedImu(t);
  m.feedEncoder(t);
  m.feedPose(t);
  m.feedLocValid(true);
}

TEST(ExceptionMonitor, NoFaultWhenHealthy)
{
  ExceptionMonitor m(defaultParams());
  for (double t = 0.0; t < 2.0; t += 0.1) {
    feedHealthy(m, t);
    EXPECT_EQ(m.check(t), FaultCode::kNone);
  }
}

TEST(ExceptionMonitor, ImuTimeout)
{
  ExceptionMonitor m(defaultParams());
  feedHealthy(m, 0.0);
  EXPECT_EQ(m.check(0.1), FaultCode::kNone);
  // IMU 停发,其余正常
  for (double t = 0.1; t < 1.0; t += 0.1) {
    m.feedEncoder(t);
    m.feedPose(t);
  }
  EXPECT_EQ(m.check(1.0), FaultCode::kImuTimeout);
}

TEST(ExceptionMonitor, EncoderTimeout)
{
  ExceptionMonitor m(defaultParams());
  feedHealthy(m, 0.0);
  for (double t = 0.1; t < 1.0; t += 0.1) {
    m.feedImu(t);
    m.feedPose(t);
  }
  EXPECT_EQ(m.check(1.0), FaultCode::kEncoderTimeout);
}

TEST(ExceptionMonitor, PoseTimeout)
{
  ExceptionMonitor m(defaultParams());
  feedHealthy(m, 0.0);
  for (double t = 0.1; t < 1.0; t += 0.1) {
    m.feedImu(t);
    m.feedEncoder(t);
  }
  EXPECT_EQ(m.check(1.0), FaultCode::kPoseTimeout);
}

TEST(ExceptionMonitor, NoTimeoutBeforeFirstData)
{
  // 从未收到数据时不报超时(等待系统启动)
  ExceptionMonitor m(defaultParams());
  EXPECT_EQ(m.check(10.0), FaultCode::kNone);
}

TEST(ExceptionMonitor, LocInvalid)
{
  ExceptionMonitor m(defaultParams());
  feedHealthy(m, 0.0);
  m.feedLocValid(false);
  EXPECT_EQ(m.check(0.1), FaultCode::kLocInvalid);
}

TEST(ExceptionMonitor, LateralDevRequiresHold)
{
  ExceptionMonitor m(defaultParams());
  m.setTrackingActive(true);
  feedHealthy(m, 0.0);
  m.feedLateralError(0.0, 0.5);   // 超阈
  // 未满持续时长:不触发
  feedHealthy(m, 0.1);
  EXPECT_EQ(m.check(0.1), FaultCode::kNone);
  // 持续超阈 0.3 s 后触发
  m.feedLateralError(0.35, 0.5);
  feedHealthy(m, 0.35);
  EXPECT_EQ(m.check(0.35), FaultCode::kLateralDev);
}

TEST(ExceptionMonitor, LateralDevRecoveredNoFault)
{
  ExceptionMonitor m(defaultParams());
  m.setTrackingActive(true);
  feedHealthy(m, 0.0);
  m.feedLateralError(0.0, 0.5);
  m.feedLateralError(0.1, 0.1);   // 回到阈内,计时清零
  m.feedLateralError(0.5, 0.5);
  feedHealthy(m, 0.5);
  EXPECT_EQ(m.check(0.5), FaultCode::kNone);
}

TEST(ExceptionMonitor, LateralDevIgnoredWhenNotTracking)
{
  ExceptionMonitor m(defaultParams());
  m.setTrackingActive(false);
  feedHealthy(m, 0.0);
  m.feedLateralError(0.0, 5.0);
  m.feedLateralError(1.0, 5.0);
  feedHealthy(m, 1.0);
  EXPECT_EQ(m.check(1.0), FaultCode::kNone);
}

TEST(ExceptionMonitor, EstopHighestPriority)
{
  ExceptionMonitor m(defaultParams());
  feedHealthy(m, 0.0);
  m.feedLocValid(false);   // 同时定位失效
  m.feedEstop(true);
  EXPECT_EQ(m.check(0.1), FaultCode::kEstop);
}

TEST(ExceptionMonitor, MissionTimeout)
{
  ExceptionParams p = defaultParams();
  p.mission_timeout = 10.0;
  ExceptionMonitor m(p);
  m.missionStarted(0.0);
  feedHealthy(m, 5.0);
  EXPECT_EQ(m.check(5.0), FaultCode::kNone);
  feedHealthy(m, 11.0);
  EXPECT_EQ(m.check(11.0), FaultCode::kMissionTimeout);
}

TEST(ExceptionMonitor, MissionEndedNoTimeout)
{
  ExceptionParams p = defaultParams();
  p.mission_timeout = 10.0;
  ExceptionMonitor m(p);
  m.missionStarted(0.0);
  m.missionEnded();
  feedHealthy(m, 20.0);
  EXPECT_EQ(m.check(20.0), FaultCode::kNone);
}

TEST(ExceptionMonitor, FaultLatchesUntilReset)
{
  ExceptionMonitor m(defaultParams());
  feedHealthy(m, 0.0);
  m.feedEstop(true);
  EXPECT_EQ(m.check(0.1), FaultCode::kEstop);
  m.feedEstop(false);
  // 解除急停后故障仍锁存
  feedHealthy(m, 0.2);
  EXPECT_EQ(m.check(0.2), FaultCode::kEstop);
  // 复位后清除
  EXPECT_TRUE(m.reset());
  feedHealthy(m, 0.3);
  EXPECT_EQ(m.check(0.3), FaultCode::kNone);
}

TEST(ExceptionMonitor, ResetRefusedWhileEstopActive)
{
  ExceptionMonitor m(defaultParams());
  feedHealthy(m, 0.0);
  m.feedEstop(true);
  m.check(0.1);
  EXPECT_FALSE(m.reset());   // 急停未解除
  m.feedEstop(false);
  EXPECT_TRUE(m.reset());
}

TEST(ExceptionMonitor, DescribeCodes)
{
  EXPECT_STREQ(tower_nav::describe(FaultCode::kNone), "正常");
  EXPECT_STREQ(tower_nav::describe(FaultCode::kEstop), "急停触发");
}

}  // namespace
