#ifndef VIW_UPDATERGPS_H
#define VIW_UPDATERGPS_H

#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <vector>

using namespace std;
using namespace Eigen;
namespace ov_type {
class Type;
}
namespace viw {
class State;
class UpdaterStatistics;
struct GPSData;
typedef vector<shared_ptr<ov_type::Type>> VEC_TYPE;
class UpdaterGPS {

public:
  /// GNSS updater
  UpdaterGPS(shared_ptr<State> state);

  /// get gps data
  void feed_measurement(const GPSData &data);

  /// find available gps data and try update
  void try_update();

  /// add keyframes at gps measurement time
  void add_keyframes(GPSData &data);

  /// status of GPS being initialized
  bool initialized = false;

  /// chi status
  vector<shared_ptr<UpdaterStatistics>> Chi;

  /// measurement timing
  map<int, deque<double>> t_hist;

protected:
  friend class Initializer;
  friend class Dynamic_Initializer;
  
  /// try initialization
  bool try_initialization();

  /// get T_WtoE initialial guess for initialization
  bool get_initial_guess(vector<GPSData> data_init, Matrix3d &RWtoE, Vector3d &pWinE);

  /// construct linear system for delayed initialization
  void construct_init_linsys(vector<GPSData> data_init, Matrix3d RWtoE, Vector3d pWinE, MatrixXd &Hx, MatrixXd &Hi, MatrixXd &R, VectorXd &res, VEC_TYPE &x_order);

  /// transform the state to ENU coordinate
  void transform_state_to_ENU();

  /// perform EKF update with GNSS measurement
  bool update(GPSData data);

  /// Stack of gnss measurements
  vector<GPSData> data_stack;

  /// State
  shared_ptr<State> state;
};
} // namespace viw
#endif // VIW_UPDATERGPS_H