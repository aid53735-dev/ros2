// TfProjection 单元测试:双向映射一致性、wrap/unwrap、不同塔参数自适应。
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "tower_nav/tf_projection.hpp"
#include "tower_nav/tower_config.hpp"

namespace
{

using tower_nav::PlaneCoord;
using tower_nav::SurfaceCoord;
using tower_nav::TfProjection;
using tower_nav::TowerConfig;
using tower_nav::TowerSegment;

std::shared_ptr<const TowerConfig> makeTower(
  double height, double bottom_d, double top_d)
{
  std::vector<TowerSegment> segs = {{0.0, height, bottom_d, top_d}};
  return std::make_shared<const TowerConfig>(
    TowerConfig::fromSegments(height, bottom_d, top_d, segs));
}

TEST(TfProjection, SurfacePlaneRoundTrip)
{
  const TfProjection proj(makeTower(80.0, 4.2, 2.6));
  for (double h : {0.0, 10.0, 40.0, 79.9}) {
    const double c = proj.circumferenceAt(h);
    for (double s : {0.0, 0.3 * c, 0.7 * c, 0.999 * c}) {
      const SurfaceCoord sc{h, s};
      const PlaneCoord pc = proj.surfaceToPlane(sc);
      EXPECT_DOUBLE_EQ(pc.x, s);
      EXPECT_DOUBLE_EQ(pc.y, h);
      const SurfaceCoord back = proj.planeToSurface(pc);
      EXPECT_NEAR(back.h, h, 1e-9);
      EXPECT_NEAR(back.s, s, 1e-9);
    }
  }
}

TEST(TfProjection, WrapXPeriodicity)
{
  const TfProjection proj(makeTower(80.0, 4.2, 2.6));
  const double y = 20.0;
  const double c = proj.circumferenceAt(y);
  EXPECT_NEAR(proj.wrapX(0.0, y), 0.0, 1e-12);
  EXPECT_NEAR(proj.wrapX(c, y), 0.0, 1e-9);
  EXPECT_NEAR(proj.wrapX(1.5 * c, y), 0.5 * c, 1e-9);
  EXPECT_NEAR(proj.wrapX(-0.25 * c, y), 0.75 * c, 1e-9);
  EXPECT_NEAR(proj.wrapX(-3.25 * c, y), 0.75 * c, 1e-9);
}

TEST(TfProjection, UnwrapNearReference)
{
  const TfProjection proj(makeTower(80.0, 4.2, 2.6));
  const double y = 20.0;
  const double c = proj.circumferenceAt(y);
  // 跨 0/周长 边界:x=0.1, ref=c-0.1 → 应展开为 c+0.1
  EXPECT_NEAR(proj.unwrapXNear(0.1, c - 0.1, y), c + 0.1, 1e-9);
  // 反向跨界
  EXPECT_NEAR(proj.unwrapXNear(c - 0.1, 0.1, y), -0.1, 1e-9);
  // 无跨界时不变
  EXPECT_NEAR(proj.unwrapXNear(1.0, 1.2, y), 1.0, 1e-9);
  // 多圈参考
  EXPECT_NEAR(proj.unwrapXNear(0.1, 5.0 * c - 0.1, y), 5.0 * c + 0.1, 1e-9);
}

TEST(TfProjection, RoundTripThroughWrap)
{
  // 展开坐标(可能多圈)→ 表面 → 展开:wrap 后一致
  const TfProjection proj(makeTower(80.0, 4.2, 2.6));
  const double y = 55.0;
  const double c = proj.circumferenceAt(y);
  const double x_multi = 3.7 * c;  // 多圈坐标
  const SurfaceCoord sc = proj.planeToSurface(PlaneCoord{x_multi, y});
  EXPECT_NEAR(sc.s, proj.wrapX(x_multi, y), 1e-9);
  const PlaneCoord pc = proj.surfaceToPlane(sc);
  // 用 unwrap 恢复连续坐标
  EXPECT_NEAR(proj.unwrapXNear(pc.x, x_multi, y), x_multi, 1e-9);
}

TEST(TfProjection, AdaptsToDifferentTower)
{
  // 更换塔筒参数,周长与映射随之变化,无需改代码
  const TfProjection proj80(makeTower(80.0, 4.2, 2.6));
  const TfProjection proj120(makeTower(120.0, 6.0, 3.0));
  EXPECT_NEAR(proj80.circumferenceAt(0.0), M_PI * 4.2, 1e-9);
  EXPECT_NEAR(proj120.circumferenceAt(0.0), M_PI * 6.0, 1e-9);
  EXPECT_NEAR(proj120.circumferenceAt(120.0), M_PI * 3.0, 1e-9);
  // 同一物理点在不同塔上 wrap 结果不同
  EXPECT_GT(
    proj120.wrapX(14.0, 0.0),
    proj80.wrapX(14.0, 0.0));
}

TEST(TfProjection, ConicalTowerCircumferenceVariesWithHeight)
{
  const TfProjection proj(makeTower(80.0, 4.2, 2.6));
  EXPECT_GT(proj.circumferenceAt(0.0), proj.circumferenceAt(80.0));
  const double mid = 0.5 * (M_PI * 4.2 + M_PI * 2.6);
  EXPECT_NEAR(proj.circumferenceAt(40.0), mid, 1e-9);
}

TEST(TfProjection, RejectsInvalidTower)
{
  EXPECT_THROW({TfProjection p(nullptr); (void)p;}, tower_nav::TowerConfigError);
  auto empty = std::make_shared<const TowerConfig>();
  EXPECT_THROW({TfProjection p(empty); (void)p;}, tower_nav::TowerConfigError);
}

TEST(TfProjection, PlaneToPoseKeepsCoordinates)
{
  const TfProjection proj(makeTower(80.0, 4.2, 2.6));
  const auto pose = proj.planeToPose(PlaneCoord{3.5, 42.0}, 1.2);
  EXPECT_DOUBLE_EQ(pose.x, 3.5);
  EXPECT_DOUBLE_EQ(pose.y, 42.0);
  EXPECT_DOUBLE_EQ(pose.yaw, 1.2);
}

}  // namespace
