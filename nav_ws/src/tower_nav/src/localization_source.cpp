// 定位源插件架构实现。
#include "tower_nav/localization_source.hpp"

namespace tower_nav
{

bool SourceRegistry::registerSource(const std::string & name, Factory factory)
{
  if (factories_.count(name) != 0) {
    return false;
  }
  factories_[name] = std::move(factory);
  return true;
}

std::shared_ptr<LocalizationSource> SourceRegistry::create(const std::string & name) const
{
  const auto it = factories_.find(name);
  if (it == factories_.end()) {
    return nullptr;
  }
  return it->second();
}

std::vector<std::string> SourceRegistry::registeredNames() const
{
  std::vector<std::string> names;
  names.reserve(factories_.size());
  for (const auto & [name, factory] : factories_) {
    names.push_back(name);
  }
  return names;
}

bool SourceRegistry::isRegistered(const std::string & name) const
{
  return factories_.count(name) != 0;
}

OdomSource::OdomSource(double wheel_radius, double wheel_separation)
: wheel_radius_(wheel_radius), wheel_separation_(wheel_separation)
{
}

void OdomSource::feedEncoder(
  double stamp, double left_wheel_vel, double right_wheel_vel)
{
  // 差速运动学正解
  const double vl = left_wheel_vel * wheel_radius_;
  const double vr = right_wheel_vel * wheel_radius_;
  SourceMeasurement m;
  m.kind = SourceMeasurement::Kind::kVelocity;
  m.stamp = stamp;
  m.v = 0.5 * (vl + vr);
  m.omega = (vr - vl) / wheel_separation_;
  m.covariance = 1e-4;
  latest_ = m;
  last_update_ = stamp;
}

std::optional<SourceMeasurement> OdomSource::latest() const
{
  return latest_;
}

void ImuSource::feedImu(double stamp, double yaw, double yaw_rate, double yaw_cov)
{
  SourceMeasurement m;
  m.kind = SourceMeasurement::Kind::kHeading;
  m.stamp = stamp;
  m.yaw = yaw;
  m.yaw_rate = yaw_rate;
  m.covariance = yaw_cov;
  latest_ = m;
  last_update_ = stamp;
}

std::optional<SourceMeasurement> ImuSource::latest() const
{
  return latest_;
}

SourceRegistry makeDefaultRegistry(double wheel_radius, double wheel_separation)
{
  SourceRegistry reg;
  reg.registerSource(
    "odom", [wheel_radius, wheel_separation]() {
      return std::make_shared<OdomSource>(wheel_radius, wheel_separation);
    });
  reg.registerSource("imu", []() {return std::make_shared<ImuSource>();});
  reg.registerSource("laser", []() {return std::make_shared<LaserSource>();});
  reg.registerSource("vision", []() {return std::make_shared<VisionSource>();});
  reg.registerSource("uwb", []() {return std::make_shared<UwbSource>();});
  return reg;
}

}  // namespace tower_nav
