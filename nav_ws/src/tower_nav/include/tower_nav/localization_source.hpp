// 插件式定位源架构:抽象 LocalizationSource 基类 + 注册工厂。
// 本期实现 OdomSource、ImuSource;预留 Laser / Vision / UWB 接口。
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace tower_nav
{

/// 定位源输出的通用测量量(按 kind 区分有效字段)
struct SourceMeasurement
{
  enum class Kind
  {
    kVelocity,   // 车体速度测量:v、omega 有效(如轮式里程计)
    kHeading,    // 航向测量:yaw、yaw_rate 有效(如 IMU)
    kPosition,   // 绝对位置测量:x、y 有效(如激光/视觉/UWB,预留)
  };

  Kind kind{Kind::kVelocity};
  double stamp{0.0};      // 测量时间 s
  double v{0.0};          // 线速度 m/s
  double omega{0.0};      // 角速度 rad/s
  double yaw{0.0};        // 航向 rad
  double yaw_rate{0.0};   // 航向角速度 rad/s
  double x{0.0};          // 绝对位置 x
  double y{0.0};          // 绝对位置 y
  double covariance{0.0};  // 主对角协方差(简化标量)
};

/// 定位源抽象基类:统一生命周期与测量读取接口
class LocalizationSource
{
public:
  virtual ~LocalizationSource() = default;

  /// 源名称(如 "odom" / "imu" / "laser")
  virtual std::string name() const = 0;

  /// 源是否可用(预留源返回 false)
  virtual bool available() const = 0;

  /// 取最新测量;无新数据时返回 nullopt
  virtual std::optional<SourceMeasurement> latest() const = 0;

  /// 最近一次数据到达时间(s);用于超时监测
  virtual double lastUpdateTime() const = 0;
};

/// 定位源注册工厂:按名称注册创建函数,支持运行时扩展
class SourceRegistry
{
public:
  using Factory = std::function<std::shared_ptr<LocalizationSource>()>;

  /// 注册一个定位源工厂;重复注册返回 false
  bool registerSource(const std::string & name, Factory factory);

  /// 创建指定名称的定位源;未注册返回 nullptr
  std::shared_ptr<LocalizationSource> create(const std::string & name) const;

  /// 已注册的源名称列表
  std::vector<std::string> registeredNames() const;

  bool isRegistered(const std::string & name) const;

private:
  std::map<std::string, Factory> factories_;
};

// ---------- 本期实现的源 ----------

/// 轮式里程计源:由编码器轮速差速解算 v、omega
class OdomSource : public LocalizationSource
{
public:
  OdomSource(double wheel_radius, double wheel_separation);

  /// 馈入一帧编码器数据(左右轮角速度 rad/s)
  void feedEncoder(double stamp, double left_wheel_vel, double right_wheel_vel);

  std::string name() const override {return "odom";}
  bool available() const override {return true;}
  std::optional<SourceMeasurement> latest() const override;
  double lastUpdateTime() const override {return last_update_;}

private:
  double wheel_radius_;
  double wheel_separation_;
  double last_update_{-1.0};
  std::optional<SourceMeasurement> latest_;
};

/// IMU 源:提供航向与角速度
class ImuSource : public LocalizationSource
{
public:
  ImuSource() = default;

  /// 馈入一帧 IMU 数据(yaw rad、yaw_rate rad/s)
  void feedImu(double stamp, double yaw, double yaw_rate, double yaw_cov);

  std::string name() const override {return "imu";}
  bool available() const override {return true;}
  std::optional<SourceMeasurement> latest() const override;
  double lastUpdateTime() const override {return last_update_;}

private:
  double last_update_{-1.0};
  std::optional<SourceMeasurement> latest_;
};

// ---------- 预留源(接口完整、注册可用,本期不可用) ----------

/// 预留:激光定位源
class LaserSource : public LocalizationSource
{
public:
  std::string name() const override {return "laser";}
  bool available() const override {return false;}  // 本期未实现
  std::optional<SourceMeasurement> latest() const override {return std::nullopt;}
  double lastUpdateTime() const override {return -1.0;}
};

/// 预留:视觉定位源
class VisionSource : public LocalizationSource
{
public:
  std::string name() const override {return "vision";}
  bool available() const override {return false;}
  std::optional<SourceMeasurement> latest() const override {return std::nullopt;}
  double lastUpdateTime() const override {return -1.0;}
};

/// 预留:UWB 定位源
class UwbSource : public LocalizationSource
{
public:
  std::string name() const override {return "uwb";}
  bool available() const override {return false;}
  std::optional<SourceMeasurement> latest() const override {return std::nullopt;}
  double lastUpdateTime() const override {return -1.0;}
};

/// 构建默认注册表:注册 odom/imu/laser/vision/uwb 全部源
SourceRegistry makeDefaultRegistry(double wheel_radius, double wheel_separation);

}  // namespace tower_nav
