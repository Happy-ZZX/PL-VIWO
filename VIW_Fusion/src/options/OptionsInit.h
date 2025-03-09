#ifndef VIW_OPTIONSINIT_H
#define VIW_OPTIONSINIT_H

#include <memory>
#include <string>
namespace ov_core {
class YamlParser;
}
namespace viw {

/**
 * @brief Struct which stores all options needed for state initialization.
 */
struct OptionsInit {

  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// Amount of time we will initialize over (seconds)
  double window_time = 1.0;

  /// Variance threshold on our acceleration to be classified as moving
  double imu_thresh = 1.0;

  /// Threshold for smoothness check of the initialization
  double imu_wheel_thresh = 0.1;

  /// boolean for state initialization using only IMU
  bool imu_only_init = false;

  /// boolean for state initialization constraining IMU z is aligned with gravity
  bool imu_gravity_aligned = false;

  /// boolean for state initialization using ground truth
  bool use_gt = false;

  /// boolean for ENU-world initialization using simulation
  bool use_gt_gnss = false;

  /// boolean to provide dense initial map to LiDAR using simulation
  bool use_gt_lidar = false;

  /// ground truth file path (It will be provided from system option if we have one)
  std::string path_gt;

  /// Size of initial state covariance
  double cov_size = 1e-4;
};
} // namespace viw

#endif // VIW_OPTIONSINIT_H
