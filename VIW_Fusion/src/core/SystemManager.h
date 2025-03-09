#ifndef VIW_SYSTEMMANAGER_H
#define VIW_SYSTEMMANAGER_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace ov_core {
class ImuData;
class CameraData;
class Feature;
class FeatureDatabase;
} // namespace ov_core

namespace viw{
struct OptionsEstimator;
class Initializer;
class Propagator;
class State;
class Simulator;
struct CamSimData;
struct GPSData;
struct STAT;
struct WheelData;
class UpdaterCamera;
class UpdaterGPS;
class UpdaterWheel;
class ZuptUpdater;
class TimeChecker;

class SystemManager {

public:
  /// Multi-sensor system handles various sensors and estimation strategies
  SystemManager(std::shared_ptr<OptionsEstimator> op);

  ~SystemManager(){};

  /// IMU measurement feeder
  bool feed_measurement_imu(const ov_core::ImuData &imu);

  /// CAM (real) measurement feeder
  void feed_measurement_camera(const ov_core::CameraData &cam);

  /// CAM (simulation) measurement feeder
  void feed_measurement_camsim(const CamSimData &cam);

  /// GPS measurement feeder
  void feed_measurement_gps(GPSData gps, bool isGeodetic);

  /// Wheel measurement feeder
  void feed_measurement_wheel(const WheelData &wheel);

  /**
   * @brief After the run has ended, print results
   */
  void visualize_final();

  /// Our master state object :D
  std::shared_ptr<State> state;

  /// Propagator of our state
  std::shared_ptr<Propagator> prop;

  /// GPS updater
  std::shared_ptr<UpdaterGPS> up_gps;

  /// Camera updater
  std::shared_ptr<UpdaterCamera> up_cam;

  /// Zero-velocity updater
  std::shared_ptr<ZuptUpdater> up_zupt;

  /// State initializer
  std::shared_ptr<Initializer> initializer;

  /// GPS datum setup
  Eigen::Vector3d gps_datum = Eigen::Vector3d::Ones() * NAN;

  /// Timing recorder for analysis
  std::shared_ptr<TimeChecker> tc_sensors;

protected:
  /// Determine next clone time
  bool get_next_clone_time(double &clone_time, double meas_t);

  /// Based on ang/lin acceleration, dynamically change cloning Hz and interpolation order
  void dynamic_cloning(int &clone_freq, int &intr_order);

  /// Compute angular and linear accelerations used for dynamic cloning
  void compute_accelerations();
  
  /// Nice print of the state and calibration results
  void print_status();

  /// Wheel updater
  std::shared_ptr<UpdaterWheel> up_whl;

  // /// ZUPT updater
  // std::shared_ptr<UpdaterZUPT> up_zupt;

  /// Average order and cloning frequency of the system
  std::shared_ptr<STAT> avg_order, avg_freq;

  /// Total distance traveled
  double distance = 0;

  /// Zupt parameter
  double last_zupt_time = 0.0;
  int zupt_detec_times = 0;
};
} //namespace viw

#endif