// LocalizationSource 插件架构单元测试:注册/创建流程、OdomSource 差速解算。
#include <cmath>

#include <gtest/gtest.h>

#include "tower_nav/localization_source.hpp"

namespace
{

using tower_nav::ImuSource;
using tower_nav::LocalizationSource;
using tower_nav::OdomSource;
using tower_nav::SourceMeasurement;
using tower_nav::SourceRegistry;

TEST(SourceRegistry, DefaultRegistryContainsAllSources)
{
  const auto reg = tower_nav::makeDefaultRegistry(0.08, 0.40);
  for (const char * name : {"odom", "imu", "laser", "vision", "uwb"}) {
    EXPECT_TRUE(reg.isRegistered(name)) << name;
    const auto src = reg.create(name);
    ASSERT_NE(src, nullptr) << name;
    EXPECT_EQ(src->name(), name);
  }
  EXPECT_EQ(reg.registeredNames().size(), 5u);
}

TEST(SourceRegistry, ImplementedSourcesAvailableReservedNot)
{
  const auto reg = tower_nav::makeDefaultRegistry(0.08, 0.40);
  EXPECT_TRUE(reg.create("odom")->available());
  EXPECT_TRUE(reg.create("imu")->available());
  // 预留源:接口完整但不可用
  EXPECT_FALSE(reg.create("laser")->available());
  EXPECT_FALSE(reg.create("vision")->available());
  EXPECT_FALSE(reg.create("uwb")->available());
  EXPECT_FALSE(reg.create("laser")->latest().has_value());
}

TEST(SourceRegistry, UnknownSourceReturnsNull)
{
  const auto reg = tower_nav::makeDefaultRegistry(0.08, 0.40);
  EXPECT_EQ(reg.create("gps"), nullptr);
  EXPECT_FALSE(reg.isRegistered("gps"));
}

TEST(SourceRegistry, DuplicateRegistrationRejected)
{
  SourceRegistry reg;
  EXPECT_TRUE(reg.registerSource("odom", []() {return std::make_shared<ImuSource>();}));
  EXPECT_FALSE(reg.registerSource("odom", []() {return std::make_shared<ImuSource>();}));
}

TEST(SourceRegistry, CustomSourceCanBeRegistered)
{
  // 验证注册机制对新源开放(扩展流程完整可用)
  class FakeUwb : public LocalizationSource
  {
  public:
    std::string name() const override {return "fake_uwb";}
    bool available() const override {return true;}
    std::optional<SourceMeasurement> latest() const override
    {
      SourceMeasurement m;
      m.kind = SourceMeasurement::Kind::kPosition;
      m.x = 1.0;
      m.y = 2.0;
      return m;
    }
    double lastUpdateTime() const override {return 0.0;}
  };

  SourceRegistry reg;
  EXPECT_TRUE(reg.registerSource("fake_uwb", []() {return std::make_shared<FakeUwb>();}));
  const auto src = reg.create("fake_uwb");
  ASSERT_NE(src, nullptr);
  EXPECT_TRUE(src->available());
  ASSERT_TRUE(src->latest().has_value());
  EXPECT_DOUBLE_EQ(src->latest()->x, 1.0);
}

TEST(OdomSource, DiffDriveForwardSolve)
{
  OdomSource src(0.08, 0.40);
  EXPECT_FALSE(src.latest().has_value());

  // 双轮同速 2.5 rad/s → v = 2.5*0.08 = 0.2 m/s, omega = 0
  src.feedEncoder(1.0, 2.5, 2.5);
  auto m = src.latest();
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->kind, SourceMeasurement::Kind::kVelocity);
  EXPECT_NEAR(m->v, 0.2, 1e-12);
  EXPECT_NEAR(m->omega, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(src.lastUpdateTime(), 1.0);

  // 差速:vl=0.08, vr=0.24 → v=0.16, omega=(0.24-0.08)/0.4=0.4
  src.feedEncoder(2.0, 1.0, 3.0);
  m = src.latest();
  EXPECT_NEAR(m->v, 0.16, 1e-12);
  EXPECT_NEAR(m->omega, 0.4, 1e-12);
}

TEST(ImuSource, HeadingMeasurement)
{
  ImuSource src;
  EXPECT_FALSE(src.latest().has_value());
  src.feedImu(3.0, 1.57, 0.1, 1e-4);
  const auto m = src.latest();
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->kind, SourceMeasurement::Kind::kHeading);
  EXPECT_DOUBLE_EQ(m->yaw, 1.57);
  EXPECT_DOUBLE_EQ(m->yaw_rate, 0.1);
  EXPECT_DOUBLE_EQ(src.lastUpdateTime(), 3.0);
}

}  // namespace
