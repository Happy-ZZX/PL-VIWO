/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "Initializer.h"
#include "feat/FeatureDatabase.h"
#include "imu/I_Initializer.h"
#include "imu_wheel/IW_Initializer.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsInit.h"
#include "options/OptionsSimulation.h"
#include "options/OptionsWheel.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "update/cam/UpdaterCamera.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "update/gps/UpdaterGPS.h"
#include "update/wheel/UpdaterWheel.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/dataset_reader.h"

using namespace std;
using namespace Eigen;
using namespace viw;
using namespace ov_core;

Initializer::Initializer(shared_ptr<State> state, PP pp_imu, UP_WHL up_whl, UP_GPS up_gps, UP_CAM up_cam)
    :pp_imu(pp_imu), up_gps(up_gps), up_whl(up_whl), up_cam(up_cam), state(state) {

  tc = make_shared<TimeChecker>();

  // Initialize with ground truth
  if (state->op->init->use_gt) {
    PRINT2(GREEN "[INIT]: Initialize with ground truth.\n" RESET);
    return;
  }

  // Initialize with IMU-wheel
  if (!state->op->init->imu_only_init && state->op->wheel->enabled) {
    PRINT2("[INIT]: Initialize with imu-wheel.\n");
    shared_ptr<IW_Initializer::IW_Initializer_Options> op = make_shared<IW_Initializer::IW_Initializer_Options>();
    op->wheel_extrinsic = state->wheel_extrinsic;
    op->wheel_intrinsic = state->wheel_intrinsic;
    op->wheel_dt = state->wheel_dt;
    op->wheel_type = state->op->wheel->type;
    op->threshold = state->op->init->imu_wheel_thresh;
    op->gravity = state->op->gravity;
    op->imu_gravity_aligned = state->op->init->imu_gravity_aligned;
    iw_init = make_shared<IW_Initializer>(op, pp_imu, up_whl);
    return;
  }

  // Initialize with IMU only
  PRINT2(GREEN "[INIT]: Initialize with imu.\n" RESET);
  i_init = make_shared<I_Initializer>(pp_imu, state->op);
}

bool Initializer::try_initializtion() {
  if (tc->counter == 0) {
    tc->ding("[INIT]: Total initialization time");
    tc->counter++;
  }
  // IMU information to be initialized
  Matrix<double, 17, 1> imustate;
  bool init_success;
  if (!state->op->init->imu_only_init && state->op->wheel->enabled) // perform IMU-Wheel initialization
    init_success = iw_init->initialization(imustate);
  else // perform IMU initialization
    init_success = i_init->initialization(imustate);

  // return if init failed
  if (!init_success) {
    delete_old_measurements();
    return false;
  }

  // Success! Set the state variables
  set_state(imustate);
  return true;
}

void Initializer::delete_old_measurements() {
  // also delete old measurements
  if (pp_imu->imu_data.back().timestamp - pp_imu->imu_data.front().timestamp > 3 * state->op->init->window_time) {
    double old_time = pp_imu->imu_data.back().timestamp - 3 * state->op->init->window_time;

    // IMU
    int del_imu = 0;
    for (auto data = pp_imu->imu_data.begin(); !pp_imu->imu_data.empty() && (*data).timestamp < old_time;) {
      del_imu++;
      data = pp_imu->imu_data.erase(data);
    }
    del_imu > 0 ? PRINT1(YELLOW "[INIT]: Delete IMU stack. Del: %d, Remain: %d\n" RESET, del_imu, pp_imu->imu_data.size()) : void();

    // Camera
    if (state->op->cam->enabled) {
      // delete camera measurements in 1/100 cam Hz because it is costy
      double cam_hz = (up_cam->t_hist.begin()->second.size() - 1) / (up_cam->t_hist.begin()->second.back() - up_cam->t_hist.begin()->second.front());
      if (last_cam_delete_t + 100.0 / cam_hz < old_time) {
        for (int i = 0; i < up_cam->trackDATABASE.size(); i++) {
          auto db = up_cam->trackDATABASE.at(i);
          auto tr = up_cam->trackFEATS.at(i)->get_feature_database();
          int db_sz = db->size();
          int tk_sz = tr->size();
          db->cleanup_measurements(old_time);
          tr->cleanup_measurements(old_time);
          db_sz -= db->size();
          tk_sz -= tr->size();
          db_sz > 0 ? PRINT1(YELLOW "[INIT]: Delete Cam%d feat DB. Del: %d, Remain: %d\n" RESET, i, db_sz, db->size()) : void();
          tk_sz > 0 ? PRINT1(YELLOW "[INIT]: Delete Cam%d trak DB. Del: %d, Remain: %d\n" RESET, i, db_sz, tr->size()) : void();
        }
        // record cam deletion time
        last_cam_delete_t = old_time;
      }
    }

    // wheel
    if (state->op->wheel->enabled) {
      int del_whl = 0;
      for (auto data = up_whl->data_stack.begin(); !up_whl->data_stack.empty() && (*data).time < old_time;) {
        del_whl++;
        data = up_whl->data_stack.erase(data);
      }
      del_whl > 0 ? PRINT1(YELLOW "[INIT]: Delete Wheel stack. Del: %d, Remain: %d\n" RESET, del_whl, up_whl->data_stack.size()) : void();
    }

    // gps
    if (state->op->gps->enabled) {
      int del_gps = 0;
      for (auto data = up_gps->data_stack.begin(); !up_gps->data_stack.empty() && (*data).time < old_time;) {
        del_gps++;
        data = up_gps->data_stack.erase(data);
      }
      del_gps > 0 ? PRINT1(YELLOW "[INIT]: Delete GNSS stack. Del: %d, Remain: %d\n" RESET, del_gps, up_gps->data_stack.size()) : void();
    }
  }
}

void Initializer::set_state(Matrix<double, 17, 1> imustate) {

  // Initialize the system
  state->imu->set_value(imustate.block(1, 0, 16, 1));
  state->imu->set_fej(imustate.block(1, 0, 16, 1));

  // Fix the global yaw and position gauge freedoms
  vector<shared_ptr<ov_type::Type>> order = {state->imu};
  MatrixXd Cov = state->op->init->cov_size * MatrixXd::Identity(state->imu->size(), state->imu->size());
  StateHelper::set_initial_covariance(state, Cov, order);

  // Make velocity uncertainty a bit bigger
  //    state->cov.block(state->imu->v()->id(), state->imu->v()->id(), 3, 3) *= 2;

  // A VIO system has 4dof unobservabile directions which can be arbitrarily picked.
  // This means that on startup, we can fix the yaw and position to be 100 percent known.
  // Thus, after determining the global to current IMU orientation after initialization, we can propagate the global error
  // into the new IMU pose. In this case the position is directly equivalent, but the orientation needs to be propagated.
  //    auto q_id = state->imu->q()->id();
  //    state->cov(q_id + 2, q_id + 2) = 0.0;
  //    state->cov.block(state->imu->p()->id(), state->imu->p()->id(), 3, 3).setZero();

  // Propagate into the current local IMU frame
  // R_GtoI = R_GtoI*R_GtoG -> H = R_GtoI
  //    Matrix3d R_GtoI = quat_2_Rot(imustate.block(1, 0, 4, 1));
  //    state->cov.block(q_id, q_id, 3, 3) = R_GtoI * state->cov.block(q_id, q_id, 3, 3).eval() * R_GtoI.transpose();

  // Set the state time
  state->time = imustate(0, 0);
  state->startup_time = imustate(0, 0);
  state->initialized = true;

  // Print what we init'ed with
  auto q = state->imu->quat();
  auto p = state->imu->pos();
  auto v = state->imu->vel();
  auto bg = state->imu->bias_g();
  auto ba = state->imu->bias_a();

  PRINT2(GREEN);
  PRINT2("[INIT]: Initialized.\n");
  tc->dong("[INIT]: Total initialization time");
  tc->print("[INIT]: Total initialization time");
  PRINT2("[INIT]: time = %.4f\n", state->time);
  PRINT2("[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n", q(0), q(1), q(2), q(3));
  PRINT2("[INIT]: position = %.4f, %.4f, %.4f\n", p(0), p(1), p(2));
  PRINT2("[INIT]: velocity = %.4f, %.4f, %.4f\n", v(0), v(1), v(2));
  PRINT2("[INIT]: bias gyro = %.4f, %.4f, %.4f\n", bg(0), bg(1), bg(2));
  PRINT2("[INIT]: bias accl = %.4f, %.4f, %.4f\n", ba(0), ba(1), ba(2));
  PRINT2(RESET);
}
