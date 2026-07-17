// TF Projection 实现:表面坐标与展开坐标双向映射、周向 wrap/unwrap。
#include "tower_nav/tf_projection.hpp"

#include <cmath>
#include <stdexcept>

namespace tower_nav
{

TfProjection::TfProjection(std::shared_ptr<const TowerConfig> tower)
: tower_(std::move(tower))
{
  if (!tower_ || !tower_->valid()) {
    throw TowerConfigError("TfProjection 需要已加载且合法的塔筒配置");
  }
}

PlaneCoord TfProjection::surfaceToPlane(const SurfaceCoord & sc) const
{
  // 展开:x = 周向弧长 s,y = 高度 h
  return PlaneCoord{sc.s, sc.h};
}

SurfaceCoord TfProjection::planeToSurface(const PlaneCoord & pc) const
{
  // 表面坐标的 s 定义在 [0, 周长) 内
  return SurfaceCoord{pc.y, wrapX(pc.x, pc.y)};
}

NavPose2D TfProjection::planeToPose(const PlaneCoord & pc, double yaw) const
{
  return NavPose2D{pc.x, pc.y, yaw};
}

double TfProjection::circumferenceAt(double y) const
{
  return tower_->circumferenceAt(y);
}

double TfProjection::wrapX(double x, double y) const
{
  const double c = circumferenceAt(y);
  double wrapped = std::fmod(x, c);
  if (wrapped < 0.0) {
    wrapped += c;
  }
  return wrapped;
}

double TfProjection::unwrapXNear(double x, double reference, double y) const
{
  const double c = circumferenceAt(y);
  // 把 x 平移到与 reference 差距不超过半周长的等价位置
  double diff = std::fmod(x - reference, c);
  if (diff > 0.5 * c) {
    diff -= c;
  } else if (diff < -0.5 * c) {
    diff += c;
  }
  return reference + diff;
}

}  // namespace tower_nav
