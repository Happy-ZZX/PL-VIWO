/*
 * VIWG_Fusion
 * Copyright (C) 2024 Zhixin Zhang
 * Copyright (C) 2024 VIWG_Fusion Contributors
 *
 * This code is implemented based on:
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VIW_Dynamic_INITIALIZER_H
#define VIW_Dynamic_INITIALIZER_H

#include "Eigen/Eigen"
#include <memory>
#include <vector>

using namespace Eigen;
using namespace std;

namespace ov_core {
class ImuData;
}

namespace ov_type {
class PoseJPL;
class Vec;
} // namespace ov_type

namespace viw {

class State;
class Propagator;
class UpdaterWheel;
class UpdaterGPS;
class UpdaterCamera;
class TimeChecker;
typedef shared_ptr<Propagator> PP;
typedef shared_ptr<UpdaterWheel> UP_WHL;
typedef shared_ptr<UpdaterGPS> UP_GPS;
typedef shared_ptr<UpdaterCamera> UP_CAM;

class Dynamic_Initializer {
public:
  /// State initializer
  Dynamic_Initializer(shared_ptr<State> state, PP pp_imu, UP_WHL up_whl, UP_GPS up_gps, UP_CAM up_cam);

  /// Initialization
  bool try_dynamic_initializtion();

private:

  /// Get IMU and wheel measurements within common window
  bool get_IMU_Wheel_data(vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);
  
  /// Delete too old measurements
  void delete_old_measurements();

  /// @brief check the visual stationary status
  /// @return stationary status
  bool visual_stationary_check ();
  
  /// check the imu stationary status
  bool imu_stationary_check(vector<ov_core::ImuData> &imu_data);

  /// imu static initialization
  bool static_initialization(Eigen::Matrix<double, 17, 1> &imustate, std::vector<ov_core::ImuData> &imu_data);
  
  /// dynamic initialization
  bool dynamic_initialization(Eigen::Matrix<double, 17, 1> &imustate, std::vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);
  
  /// Compute gravity direction by solving constrained problem
  bool init_gI_dongsi(Vector3d bg, Vector3d v_I0inI0, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data, Vector3d &gravity_inI0);
  
  /// Compute IMU ba given all the other information
  Vector3d init_ba(Vector3d bg, Vector3d v_I0inI0, Vector3d g_inI0, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);
  
  /// Compute linear velocity residual for sanity check
  VectorXd residual(Vector3d bg, Vector3d ba, Vector3d v_I0inI0, Vector3d g_inI0, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);
  
  /// Gram-Schmidt method to recover global rotation  
  Matrix3d gram_schmidt(const Vector3d &gravity_inI);

  /// IMU orientation propagation
  static void IMU_prop_rk4(double dt, const Vector3d &w_hat1, const Vector3d &w_hat2, Vector4d &new_q);

  /// Get coefficients for solving constrained problem
  static VectorXd compute_dongsi_coeff(MatrixXd &D, const MatrixXd &d, double gravity_mag);

  /// sensor stationary
  bool visual_stationary = true;
  bool wheel_stationary = true;
  bool imu_stationary = true;

  // /// Initialization methods
  // shared_ptr<Static_Initializer> static_init;
  // shared_ptr<Slow_Initializer> slow_init;
  // shared_ptr<Aggressive_Initializer> aggressive_init;

  /// Each sensor handler
  shared_ptr<Propagator> imu_pp;
  shared_ptr<UpdaterGPS> gps_up;
  shared_ptr<UpdaterWheel> wheel_up;
  shared_ptr<UpdaterCamera> cam_up;

  /// Time recorder
  shared_ptr<TimeChecker> tc;

  /// Max paralle change (m) and velocity (m/s) that we should consider stationary
  double stationary_max_paralle = 0.01;
  double stationary_max_velocity = 0.2;
  double stationary_max_variance = 0.1;
  int init_max_disparity = 10;

  /// state
  shared_ptr<State> state;
  
  // wheel type 
  string wheel_type;

  // wheel calibration parameters
  Matrix3d R_OtoI;
  Vector3d p_IinO;
  double toff;
  double rl;
  double rr;
  double base_length;

  // Last time we removed old sensor information
  double last_cam_delete_t = -1;
};
} // namespace viw
#endif // VIW_Dynamic_INITIALIZER_H