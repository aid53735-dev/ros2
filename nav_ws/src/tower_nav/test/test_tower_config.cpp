// TowerConfig 单元测试:合法/非法配置校验、r(h) 插值。
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "tower_nav/tower_config.hpp"

namespace
{

using tower_nav::TowerConfig;
using tower_nav::TowerConfigError;
using tower_nav::TowerSegment;

// 合法三段塔 YAML
const char * kValidYaml = R"(
tower:
  total_height: 80.0
  bottom_diameter: 4.2
  top_diameter: 2.6
  segments:
    - {start_height: 0.0, end_height: 30.0, bottom_diameter: 4.2, top_diameter: 3.8}
    - {start_height: 30.0, end_height: 60.0, bottom_diameter: 3.8, top_diameter: 3.1}
    - {start_height: 60.0, end_height: 80.0, bottom_diameter: 3.1, top_diameter: 2.6}
)";

TEST(TowerConfig, LoadValidYaml)
{
  const TowerConfig cfg = TowerConfig::loadFromString(kValidYaml);
  EXPECT_TRUE(cfg.valid());
  EXPECT_DOUBLE_EQ(cfg.totalHeight(), 80.0);
  EXPECT_DOUBLE_EQ(cfg.bottomDiameter(), 4.2);
  EXPECT_DOUBLE_EQ(cfg.topDiameter(), 2.6);
  EXPECT_EQ(cfg.segments().size(), 3u);
}

TEST(TowerConfig, RadiusInterpolation)
{
  const TowerConfig cfg = TowerConfig::loadFromString(kValidYaml);
  // 边界值
  EXPECT_NEAR(cfg.radiusAt(0.0), 2.1, 1e-12);
  EXPECT_NEAR(cfg.radiusAt(80.0), 1.3, 1e-12);
  // 分段交界
  EXPECT_NEAR(cfg.radiusAt(30.0), 1.9, 1e-12);
  EXPECT_NEAR(cfg.radiusAt(60.0), 1.55, 1e-12);
  // 段内中点线性插值
  EXPECT_NEAR(cfg.radiusAt(15.0), 0.5 * (4.2 + 3.8) / 2.0, 1e-12);
  EXPECT_NEAR(cfg.radiusAt(45.0), 0.5 * (3.8 + 3.1) / 2.0, 1e-12);
  EXPECT_NEAR(cfg.radiusAt(70.0), 0.5 * (3.1 + 2.6) / 2.0, 1e-12);
}

TEST(TowerConfig, RadiusClampOutOfRange)
{
  const TowerConfig cfg = TowerConfig::loadFromString(kValidYaml);
  EXPECT_NEAR(cfg.radiusAt(-5.0), cfg.radiusAt(0.0), 1e-12);
  EXPECT_NEAR(cfg.radiusAt(100.0), cfg.radiusAt(80.0), 1e-12);
}

TEST(TowerConfig, CircumferenceMatchesRadius)
{
  const TowerConfig cfg = TowerConfig::loadFromString(kValidYaml);
  EXPECT_NEAR(cfg.circumferenceAt(15.0), 2.0 * M_PI * cfg.radiusAt(15.0), 1e-12);
}

TEST(TowerConfig, RejectNegativeHeight)
{
  const char * yaml = R"(
tower:
  total_height: -80.0
  bottom_diameter: 4.2
  top_diameter: 2.6
  segments:
    - {start_height: 0.0, end_height: 80.0, bottom_diameter: 4.2, top_diameter: 2.6}
)";
  EXPECT_THROW(TowerConfig::loadFromString(yaml), TowerConfigError);
}

TEST(TowerConfig, RejectNonPositiveDiameter)
{
  const char * yaml = R"(
tower:
  total_height: 80.0
  bottom_diameter: 0.0
  top_diameter: 2.6
  segments:
    - {start_height: 0.0, end_height: 80.0, bottom_diameter: 0.0, top_diameter: 2.6}
)";
  EXPECT_THROW(TowerConfig::loadFromString(yaml), TowerConfigError);
}

TEST(TowerConfig, RejectGapBetweenSegments)
{
  const char * yaml = R"(
tower:
  total_height: 80.0
  bottom_diameter: 4.2
  top_diameter: 2.6
  segments:
    - {start_height: 0.0, end_height: 30.0, bottom_diameter: 4.2, top_diameter: 3.8}
    - {start_height: 40.0, end_height: 80.0, bottom_diameter: 3.8, top_diameter: 2.6}
)";
  EXPECT_THROW(TowerConfig::loadFromString(yaml), TowerConfigError);
}

TEST(TowerConfig, RejectIncompleteCoverage)
{
  const char * yaml = R"(
tower:
  total_height: 80.0
  bottom_diameter: 4.2
  top_diameter: 2.6
  segments:
    - {start_height: 0.0, end_height: 70.0, bottom_diameter: 4.2, top_diameter: 2.6}
)";
  EXPECT_THROW(TowerConfig::loadFromString(yaml), TowerConfigError);
}

TEST(TowerConfig, RejectDiameterMismatchAtJoint)
{
  const char * yaml = R"(
tower:
  total_height: 80.0
  bottom_diameter: 4.2
  top_diameter: 2.6
  segments:
    - {start_height: 0.0, end_height: 30.0, bottom_diameter: 4.2, top_diameter: 3.8}
    - {start_height: 30.0, end_height: 80.0, bottom_diameter: 3.5, top_diameter: 2.6}
)";
  EXPECT_THROW(TowerConfig::loadFromString(yaml), TowerConfigError);
}

TEST(TowerConfig, RejectEmptySegments)
{
  const char * yaml = R"(
tower:
  total_height: 80.0
  bottom_diameter: 4.2
  top_diameter: 2.6
  segments: []
)";
  EXPECT_THROW(TowerConfig::loadFromString(yaml), TowerConfigError);
}

TEST(TowerConfig, RejectMissingField)
{
  const char * yaml = R"(
tower:
  total_height: 80.0
  bottom_diameter: 4.2
)";
  EXPECT_THROW(TowerConfig::loadFromString(yaml), TowerConfigError);
}

TEST(TowerConfig, RejectMalformedYaml)
{
  EXPECT_THROW(TowerConfig::loadFromString("tower: ["), TowerConfigError);
}

TEST(TowerConfig, RejectMissingFile)
{
  EXPECT_THROW(TowerConfig::loadFromFile("/nonexistent/tower.yaml"), TowerConfigError);
}

TEST(TowerConfig, UnorderedSegmentsAreSorted)
{
  // 分段乱序给入,应排序后校验通过
  std::vector<TowerSegment> segs = {
    {60.0, 80.0, 3.1, 2.6},
    {0.0, 30.0, 4.2, 3.8},
    {30.0, 60.0, 3.8, 3.1},
  };
  const TowerConfig cfg = TowerConfig::fromSegments(80.0, 4.2, 2.6, segs);
  EXPECT_NEAR(cfg.radiusAt(45.0), 0.5 * (3.8 + 3.1) / 2.0, 1e-12);
}

TEST(TowerConfig, QueryBeforeLoadThrows)
{
  const TowerConfig cfg;
  EXPECT_THROW(cfg.radiusAt(1.0), TowerConfigError);
}

}  // namespace
