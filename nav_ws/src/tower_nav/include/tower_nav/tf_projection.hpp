// TF Projection 坐标投影公共库:
// 塔筒表面坐标 (h, s) ↔ 导航二维展开坐标 (x=周向弧长, y=高度) ↔ Navigation Pose。
// 映射关系由 TowerConfig 几何模型驱动,随 tower.yaml 自适应。
#pragma once

#include <memory>

#include "tower_nav/tower_config.hpp"

namespace tower_nav
{

/// 塔筒表面坐标:高度 h(m)+ 周向弧长 s(m,绕塔一周为 2πr(h))
struct SurfaceCoord
{
  double h{0.0};
  double s{0.0};
};

/// 导航二维展开坐标:x = 周向弧长(可为 unwrapped 连续值),y = 高度
struct PlaneCoord
{
  double x{0.0};
  double y{0.0};
};

/// 展开面内导航位姿:位置 + 航向角
struct NavPose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// 坐标投影:双向映射,周向环绕处理
class TfProjection
{
public:
  explicit TfProjection(std::shared_ptr<const TowerConfig> tower);

  /// 表面坐标 → 展开坐标(恒等映射的显式表达:x=s, y=h)
  PlaneCoord surfaceToPlane(const SurfaceCoord & sc) const;

  /// 展开坐标 → 表面坐标(s 归一化到 [0, 周长))
  SurfaceCoord planeToSurface(const PlaneCoord & pc) const;

  /// 展开坐标 → 导航位姿(附加航向)
  NavPose2D planeToPose(const PlaneCoord & pc, double yaw) const;

  /// 高度 y 处把周向坐标 x 归一化(wrap)到 [0, 周长)
  double wrapX(double x, double y) const;

  /// 以 reference 为基准把 x 展开(unwrap)为与其最近的连续值:
  /// 结果与 reference 之差不超过半周长。用于误差计算跨越 0/周长 边界时。
  double unwrapXNear(double x, double reference, double y) const;

  /// 高度 y 处的周长
  double circumferenceAt(double y) const;

  /// 底层塔筒几何
  const TowerConfig & tower() const {return *tower_;}

private:
  std::shared_ptr<const TowerConfig> tower_;
};

}  // namespace tower_nav
