// 塔筒参数配置实现:YAML 加载、校验、r(h) 插值。
#include "tower_nav/tower_config.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace tower_nav
{

namespace
{
constexpr double kGeomTol = 1e-6;  // 几何一致性容差(m)

TowerConfig parseNode(const YAML::Node & root)
{
  if (!root["tower"]) {
    throw TowerConfigError("tower.yaml 缺少顶层 'tower' 键");
  }
  const YAML::Node t = root["tower"];
  for (const char * key :
    {"total_height", "bottom_diameter", "top_diameter", "segments"})
  {
    if (!t[key]) {
      throw TowerConfigError(std::string("tower.yaml 缺少字段: ") + key);
    }
  }

  std::vector<TowerSegment> segments;
  for (const auto & s : t["segments"]) {
    TowerSegment seg;
    seg.start_height = s["start_height"].as<double>();
    seg.end_height = s["end_height"].as<double>();
    seg.bottom_diameter = s["bottom_diameter"].as<double>();
    seg.top_diameter = s["top_diameter"].as<double>();
    segments.push_back(seg);
  }

  return TowerConfig::fromSegments(
    t["total_height"].as<double>(),
    t["bottom_diameter"].as<double>(),
    t["top_diameter"].as<double>(),
    segments);
}
}  // namespace

TowerConfig TowerConfig::loadFromFile(const std::string & yaml_path)
{
  try {
    return parseNode(YAML::LoadFile(yaml_path));
  } catch (const YAML::Exception & e) {
    throw TowerConfigError(std::string("tower.yaml 解析失败: ") + e.what());
  }
}

TowerConfig TowerConfig::loadFromString(const std::string & yaml_content)
{
  try {
    return parseNode(YAML::Load(yaml_content));
  } catch (const YAML::Exception & e) {
    throw TowerConfigError(std::string("tower.yaml 解析失败: ") + e.what());
  }
}

TowerConfig TowerConfig::fromSegments(
  double total_height, double bottom_diameter, double top_diameter,
  const std::vector<TowerSegment> & segments)
{
  TowerConfig cfg;
  cfg.total_height_ = total_height;
  cfg.bottom_diameter_ = bottom_diameter;
  cfg.top_diameter_ = top_diameter;
  cfg.segments_ = segments;
  // 按起始高度排序,便于连续性校验与查找
  std::sort(
    cfg.segments_.begin(), cfg.segments_.end(),
    [](const TowerSegment & a, const TowerSegment & b) {
      return a.start_height < b.start_height;
    });
  cfg.validate();
  cfg.valid_ = true;
  return cfg;
}

void TowerConfig::validate() const
{
  auto fail = [](const std::string & msg) {
      throw TowerConfigError("塔筒配置校验失败: " + msg);
    };

  if (!(total_height_ > 0.0)) {
    fail("total_height 必须为正");
  }
  if (!(bottom_diameter_ > 0.0) || !(top_diameter_ > 0.0)) {
    fail("bottom_diameter / top_diameter 必须为正");
  }
  if (segments_.empty()) {
    fail("segments 不能为空");
  }

  for (size_t i = 0; i < segments_.size(); ++i) {
    const auto & s = segments_[i];
    std::ostringstream tag;
    tag << "segments[" << i << "] ";
    if (!(s.bottom_diameter > 0.0) || !(s.top_diameter > 0.0)) {
      fail(tag.str() + "直径必须为正");
    }
    if (!(s.end_height > s.start_height)) {
      fail(tag.str() + "end_height 必须大于 start_height");
    }
    if (s.start_height < -kGeomTol) {
      fail(tag.str() + "start_height 不能为负");
    }
  }

  // 连续且完整覆盖 0 ~ total_height
  if (std::abs(segments_.front().start_height) > kGeomTol) {
    fail("首段 start_height 必须为 0");
  }
  if (std::abs(segments_.back().end_height - total_height_) > kGeomTol) {
    fail("末段 end_height 必须等于 total_height");
  }
  for (size_t i = 1; i < segments_.size(); ++i) {
    if (std::abs(segments_[i].start_height - segments_[i - 1].end_height) > kGeomTol) {
      std::ostringstream oss;
      oss << "segments[" << i - 1 << "] 与 segments[" << i << "] 高度不连续";
      fail(oss.str());
    }
    if (std::abs(segments_[i].bottom_diameter - segments_[i - 1].top_diameter) > kGeomTol) {
      std::ostringstream oss;
      oss << "segments[" << i - 1 << "] 顶径与 segments[" << i << "] 底径不衔接";
      fail(oss.str());
    }
  }

  // 首末段直径与全局参数一致
  if (std::abs(segments_.front().bottom_diameter - bottom_diameter_) > kGeomTol) {
    fail("首段 bottom_diameter 与全局 bottom_diameter 不一致");
  }
  if (std::abs(segments_.back().top_diameter - top_diameter_) > kGeomTol) {
    fail("末段 top_diameter 与全局 top_diameter 不一致");
  }
}

double TowerConfig::radiusAt(double h) const
{
  if (!valid_) {
    throw TowerConfigError("塔筒配置未加载,无法查询半径");
  }
  // 越界钳位
  h = std::clamp(h, 0.0, total_height_);
  // 定位所在分段(分段已按高度排序且连续)
  for (const auto & s : segments_) {
    if (h <= s.end_height + kGeomTol) {
      const double span = s.end_height - s.start_height;
      const double ratio = std::clamp((h - s.start_height) / span, 0.0, 1.0);
      const double d = s.bottom_diameter + ratio * (s.top_diameter - s.bottom_diameter);
      return 0.5 * d;
    }
  }
  // 理论不可达(h 已钳位到末段之内)
  return 0.5 * segments_.back().top_diameter;
}

double TowerConfig::circumferenceAt(double h) const
{
  return 2.0 * M_PI * radiusAt(h);
}

}  // namespace tower_nav
