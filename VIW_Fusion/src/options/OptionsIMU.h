#ifndef VIW_OPTIONSIMU_H
#define VIW_OPTIONSIMU_H

#include <memory>
#include <string>

namespace ov_core {
class YamlParser;
}
namespace viw {

/**
 * @brief Struct which stores all IMU options needed for state estimation.
 */
struct OptionsIMU {

  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// rostopic to subscribe
  std::string topic;

  /// IMU noise (gyroscope and accelerometer)
  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-03;

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;
};
} // namespace viw
#endif // VIW_OPTIONSIMU_H