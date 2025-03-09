/*
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

#ifndef VIW_INITIALIZER_H
#define VIW_INITIALIZER_H

#include "Eigen/Eigen"
#include <memory>

using namespace std;

namespace viw {

class State;
class Propagator;
class UpdaterWheel;
class UpdaterGPS;
class UpdaterCamera;
class IW_Initializer;
class I_Initializer;
class TimeChecker;
typedef shared_ptr<Propagator> PP;
typedef shared_ptr<UpdaterWheel> UP_WHL;
typedef shared_ptr<UpdaterGPS> UP_GPS;
typedef shared_ptr<UpdaterCamera> UP_CAM;

class Initializer {
public:
  /// State initializer
  Initializer(shared_ptr<State> state, PP pp_imu, UP_WHL up_whl, UP_GPS up_gps, UP_CAM up_cam);

  /// Initialization
  bool try_initializtion();

private:

  /// Delete too old measurements
  void delete_old_measurements();

  /// Set state after initialization success
  void set_state(Eigen::Matrix<double, 17, 1> imustate);

  /// Initialization methods
  shared_ptr<IW_Initializer> iw_init;
  shared_ptr<I_Initializer> i_init;

  /// Each sensor handler
  shared_ptr<Propagator> pp_imu;
  shared_ptr<UpdaterGPS> up_gps;
  shared_ptr<UpdaterWheel> up_whl;
  shared_ptr<UpdaterCamera> up_cam;

  /// Last time we removed old sensor information
  double last_cam_delete_t = -1;

  /// Time recorder
  shared_ptr<TimeChecker> tc;

  /// state
  shared_ptr<State> state;
};
} // namespace viw

#endif // VIW_INITIALIZER_H
