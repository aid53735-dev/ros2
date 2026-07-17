// 塔筒参数配置:加载 tower.yaml、合法性校验、r(h) 分段线性插值几何模型。
// 严禁在代码中硬编码塔筒尺寸,所有几何均来自 YAML。
#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace tower_nav
{

/// 塔筒配置加载/校验异常
class TowerConfigError : public std::runtime_error
{
public:
  explicit TowerConfigError(const std::string & msg)
  : std::runtime_error(msg) {}
};

/// 单个锥形分段:[start_height, end_height],底/顶直径线性过渡
struct TowerSegment
{
  double start_height{0.0};
  double end_height{0.0};
  double bottom_diameter{0.0};
  double top_diameter{0.0};
};

/// 塔筒几何模型:半径随高度分段线性插值
class TowerConfig
{
public:
  TowerConfig() = default;

  /// 从 YAML 文件加载并校验;失败抛出 TowerConfigError
  static TowerConfig loadFromFile(const std::string & yaml_path);

  /// 从 YAML 字符串加载(便于测试)
  static TowerConfig loadFromString(const std::string & yaml_content);

  /// 直接从字段构造并校验(便于测试与程序化配置)
  static TowerConfig fromSegments(
    double total_height, double bottom_diameter, double top_diameter,
    const std::vector<TowerSegment> & segments);

  /// 高度 h 处的塔筒半径(m);h 越界时钳到 [0, total_height]
  double radiusAt(double h) const;

  /// 高度 h 处的周长(m)
  double circumferenceAt(double h) const;

  double totalHeight() const {return total_height_;}
  double bottomDiameter() const {return bottom_diameter_;}
  double topDiameter() const {return top_diameter_;}
  const std::vector<TowerSegment> & segments() const {return segments_;}
  bool valid() const {return valid_;}

private:
  /// 合法性校验:数值为正、分段按高度排序连续、完整覆盖 0~total_height、
  /// 相邻段直径衔接一致、首末段直径与全局 bottom/top 一致。
  void validate() const;

  double total_height_{0.0};
  double bottom_diameter_{0.0};
  double top_diameter_{0.0};
  std::vector<TowerSegment> segments_;
  bool valid_{false};
};

}  // namespace tower_nav
