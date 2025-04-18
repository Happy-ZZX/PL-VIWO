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

#include "Dynamic_Initializer.h"
#include "options/OptionsWheel.h"
#include "options/Options.h"
#include "options/OptionsInit.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "state/State.h"
#include "state/Propagator.h"
#include "state/StateHelper.h"
#include "types/PoseJPL.h"
#include "update/cam/UpdaterCamera.h"
#include "update/wheel/UpdaterWheel.h"
#include "update/wheel/WheelTypes.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "update/gps/UpdaterGPS.h"

#include "feat/FeatureHelper.h"
#include "feat/FeatureDatabase.h"
#include "track/TrackBase.h"
#include "utils/Print_Logger.h"
#include "utils/colors.h"
#include "utils/sensor_data.h"
#include "utils/TimeChecker.h"

using namespace std;
using namespace Eigen;
using namespace viw;
using namespace ov_core;

Dynamic_Initializer::Dynamic_Initializer(shared_ptr<State> state, PP imu_pp, UP_WHL wheel_up, UP_GPS gps_up, UP_CAM cam_up)
    :imu_pp(imu_pp), gps_up(gps_up), wheel_up(wheel_up), cam_up(cam_up), state(state) {
  
  tc = make_shared<TimeChecker>();

  //Load wheel calibration parameters
  PRINT2("[INIT]: Dynamic Initialize.\n");
  wheel_type = state->op->wheel->type;
  R_OtoI = state->wheel_extrinsic->Rot().transpose();
  p_IinO = state->wheel_extrinsic->pos();
  toff = state->wheel_dt->value()(0);
  rl = state->wheel_intrinsic->value()(0);
  rr = state->wheel_intrinsic->value()(1);
  base_length = state->wheel_intrinsic->value()(2);
  
}

bool Dynamic_Initializer::try_dynamic_initializtion() {
  // load data
  std::vector<ov_core::ImuData> imu_data;
  std::vector<pair<double, VectorXd>> wheel_data;
  if (!get_IMU_Wheel_data(imu_data, wheel_data))
    return false;
  // Try to initialize the system
  // We will wait for a jerk if we do not have the zero velocity update enabled
  // Otherwise we can initialize right away as the zero velocity will handle the stationary case
  //bool wait_for_jerk = (ZUPT_up == nullptr);

  // wheel stationary detection
  bool wheel_stationary = true;
  for (const auto &wheel : wheel_data) {
    if (wheel.second.norm() > 0) {
      wheel_stationary = false;
      break;
    }
  }

  // visual stationary detection 
  bool imu_stationary = imu_stationary_check(imu_data);

  // imu stationary detection
  bool visual_stationary = visual_stationary_check();

  // now try to initialize 
  Matrix<double, 17, 1> imustate;
  bool success;

  std::vector<std::shared_ptr<ov_type::Type>> order;
  Eigen::MatrixXd covariance;

  // if two of the stationary check passed, then Statistic Initialize
  if ((imu_stationary && wheel_stationary) || (imu_stationary && visual_stationary) || (wheel_stationary && visual_stationary)) {
    PRINT2("[INIT]: Statistic Initialize.\n" RESET);
    success = static_initialization(imustate, imu_data);
  }
  else{
    PRINT2("[INIT]: Dynamic Initialize.\n" RESET);
    success = dynamic_initialization(imustate, imu_data, wheel_data);
  }

  // return if failed
  if (!success){
    delete_old_measurements();
    return false;
  }
  else
    return true;
}

bool Dynamic_Initializer::get_IMU_Wheel_data(vector<ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data) {

  // do not perform initialization if we don't have enough data
  if (imu_pp->imu_data.size() < 3 || wheel_up->data_stack.size() < 3) {
    PRINT1(YELLOW "[IW-Init]: Waiting for collecting IMU(%d < 3) and wheel data(%d < 3).\n" RESET, imu_pp->imu_data.size(), wheel_up->data_stack.size());
    return false;
  }
  
  // get the max times of imu and wheel data
  // double min_imu_t = imu_pp->imu_data.at(1).timestamp;
  double max_imu_t = imu_pp->imu_data.at(imu_pp->imu_data.size() - 2).timestamp;
  // double min_wheel_t = wheel_up->data_stack.at(1).time + toff;                               // convert to IMU time
  double max_wheel_t = wheel_up->data_stack.at(wheel_up->data_stack.size() - 2).time + toff; // convert to IMU time

  // get the min max time
  double newest_time = min(max_imu_t, max_wheel_t);
  double oldest_time = newest_time - state->op->init->window_time - 0.10;

  // get each data within common timezone (up to timeoffset)
  if (!imu_pp->select_imu_readings(oldest_time, newest_time, imu_data)) {
    PRINT1(YELLOW "[IW-Init]: Failed to get IMU measurements between %.4f - %.4f.\n" RESET, oldest_time, newest_time);
    return false;
  }

  vector<WheelData> wheel_raw_data;
  if (!wheel_up->select_wheel_data(oldest_time - toff, newest_time - toff, wheel_raw_data)) {
    PRINT1(YELLOW "[IW-Init]: Failed to get wheel measurements between %.4f - %.4f.\n" RESET, oldest_time - toff, newest_time - toff);
    return false;
  }

  // do not perform initialization if we don't have enough data 2
  if (imu_data.size() < 20 || wheel_raw_data.size() < 20) {
    PRINT1(YELLOW "[IW-Init]: Not enough IMU (%d / 20) and wheel measurements (%d / 20).\n" RESET, imu_data.size(), wheel_raw_data.size());
    return false;
  }

  // Convert the wheel data to ang/lin velocities
  for (auto wheel : wheel_raw_data) {
    // compute the angular velocity at the odometry frame
    Vector3d w_OinO, v_OinO;
    if (wheel_type == "Wheel2DAng" || wheel_type == "Wheel3DAng") {
      w_OinO << 0, 0, (wheel.m2 * rr - wheel.m1 * rl) / base_length;
      v_OinO << (wheel.m2 * rr + wheel.m1 * rl) / 2, 0, 0;
    } else if (wheel_type == "Wheel2DLin" || wheel_type == "Wheel3DLin") {
      w_OinO << 0, 0, (wheel.m2 - wheel.m1) / base_length;
      v_OinO << (wheel.m2 + wheel.m1) / 2, 0, 0;
    } else if (wheel_type == "Wheel2DCen" || wheel_type == "Wheel3DCen") {
      w_OinO << 0, 0, wheel.m1;
      v_OinO << wheel.m2, 0, 0;
    } else {
      PRINT4("Wrong wheel type selected!");
      exit(EXIT_FAILURE);
    }

    // append the velocities
    VectorXd wv = VectorXd::Zero(6);
    wv.block(0, 0, 3, 1) = w_OinO;
    wv.block(3, 0, 3, 1) = v_OinO;
    wheel_data.emplace_back(wheel.time, wv);
  }
  return true;
}

void Dynamic_Initializer::delete_old_measurements() {
  // also delete old measurements
  if (imu_pp->imu_data.back().timestamp - imu_pp->imu_data.front().timestamp > 3 * state->op->init->window_time) {
    double old_time = imu_pp->imu_data.back().timestamp - 3 * state->op->init->window_time;

    // IMU
    int del_imu = 0;
    for (auto data = imu_pp->imu_data.begin(); !imu_pp->imu_data.empty() && (*data).timestamp < old_time;) {
      del_imu++;
      data = imu_pp->imu_data.erase(data);
    }
    del_imu > 0 ? PRINT1(YELLOW "[INIT]: Delete IMU stack. Del: %d, Remain: %d\n" RESET, del_imu, imu_pp->imu_data.size()) : void();

    // Camera
    if (state->op->cam->enabled) {
      // delete camera measurements in 1/100 cam Hz because it is costy
      double cam_hz = (cam_up->t_hist.begin()->second.size() - 1) / (cam_up->t_hist.begin()->second.back() - cam_up->t_hist.begin()->second.front());
      if (last_cam_delete_t + 100.0 / cam_hz < old_time) {
        for (int i = 0; i < cam_up->trackDATABASE.size(); i++) {
          auto db = cam_up->trackDATABASE.at(i);
          auto tr = cam_up->trackFEATS.at(i)->get_feature_database();
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
      for (auto data = wheel_up->data_stack.begin(); !wheel_up->data_stack.empty() && (*data).time < old_time;) {
        del_whl++;
        data = wheel_up->data_stack.erase(data);
      }
      del_whl > 0 ? PRINT1(YELLOW "[INIT]: Delete Wheel stack. Del: %d, Remain: %d\n" RESET, del_whl, wheel_up->data_stack.size()) : void();
    }

    // gps
    if (state->op->gps->enabled) {
      int del_gps = 0;
      for (auto data = gps_up->data_stack.begin(); !gps_up->data_stack.empty() && (*data).time < old_time;) {
        del_gps++;
        data = gps_up->data_stack.erase(data);
      }
      del_gps > 0 ? PRINT1(YELLOW "[INIT]: Delete GNSS stack. Del: %d, Remain: %d\n" RESET, del_gps, gps_up->data_stack.size()) : void();
    }
  }
}

bool Dynamic_Initializer::visual_stationary_check (){
  // get the database
  map<int, shared_ptr<ov_core::TrackBase>> trackDATABASE = cam_up->get_trackFEATS();
  
  // Get the newest and oldest timestamps we will try to initialize between!
  double newest_cam_time = -1;
  double oldest_time = 100;
  for (auto const &db : trackDATABASE){
    for (auto const &feat : db.second->get_feature_database()->get_internal_data()) {
      for (auto const &camtimepair : feat.second->timestamps) {
        for (auto const &time : camtimepair.second) {
          newest_cam_time = std::max(newest_cam_time, time);
        }
      }
    }
  }
  
  // oldest time equal to the initialize window size
  oldest_time = newest_cam_time - state->op->init->window_time - 0.10;

  if (newest_cam_time < 0 || oldest_time < 0) {
    return false;
  }
  
  // Compute the disparity of each camera at the current timestep
  // If disparity is zero or negative we will always use the static initializer
  bool disparity_detected_moving = false;
  if (init_max_disparity > 0) {

    // Get the disparity statistics from this image to the previous
    for (auto const &_db : trackDATABASE){
      double avg_disp, var_disp;
      int num_features = 0;
      FeatureHelper::compute_disparity(_db.second->get_feature_database(), avg_disp, var_disp, num_features,newest_cam_time, oldest_time);
    
      // Return if we can't compute the disparity
      int feat_thresh = 15;
      if (num_features < feat_thresh) {
        PRINT1(YELLOW "[init]: not enough feats to compute disp: %d < %d\n" RESET, num_features, feat_thresh);
        return false;
      }

      // Check if it passed our check!
      PRINT1(YELLOW "[init]: disparity is %.3f (%.2f thresh)\n" RESET, avg_disp, init_max_disparity);
      disparity_detected_moving = (avg_disp > init_max_disparity);
    }
  }

  return !disparity_detected_moving;
}

bool Dynamic_Initializer::imu_stationary_check(vector<ImuData> &imu_data){
 
  // Return if we don't have any imu data yet
  if (imu_data.empty()) {
    return false;
  }
  
  bool imu_stationary = false;
  
  // First calculate the preintegration stationary
  bool preint_stationary = false;
  
  // Calculate the delta P form the imu data
  Eigen::Vector3d dv_imu = {0, 0, 0};
  Eigen::Vector3d dp_imu = {0, 0, 0};
  Eigen::Matrix3d dR_imu = MatrixXd::Identity(3, 3);
  Eigen::Vector4d dq_imu = {0, 0, 0, 1};
  double sum_t;
  Vector3d sum_a = imu_data[0].am;
  for (unsigned int i = 0; i+1 < imu_data.size(); i++){
    
    // Time elapsed over interval
    ov_core::ImuData data_minus = imu_data[i];
    ov_core::ImuData data_plus = imu_data[i+1];
    double dt = data_plus.timestamp - data_minus.timestamp;
    
    // Corrected imu acc measurements without biases
    Eigen::Vector3d a_hat1 = data_minus.am;
    Eigen::Vector3d a_hat2 = data_plus.am;
    Eigen::Vector3d a_hat = 0.5 * (a_hat1 + a_hat2);

    // Corrected imu gyro measurements without biases
    Eigen::Vector3d w_hat1 = data_minus.wm;
    Eigen::Vector3d w_hat2 = data_plus.wm;
    Eigen::Vector3d w_hat = 0.5 * (w_hat1 + w_hat2);
    
    //
    sum_a = sum_a + imu_data[i+1].am;

    // Propagate the mean forward
    dp_imu = dp_imu + dv_imu * dt + 0.5 * dR_imu.transpose() * a_hat * dt * dt - 0.5 * state->op->gravity * dt * dt;
    dv_imu = dv_imu + dR_imu.transpose() * a_hat * dt - state->op->gravity * dt;
    dR_imu = exp_so3(-w_hat * dt) * dR_imu;
    dq_imu = rot_2_quat(dR_imu);
    sum_t += dt;
  }
  
  if (dp_imu.norm() < stationary_max_paralle && dv_imu.norm() < stationary_max_velocity)
    preint_stationary = true;
  else
    preint_stationary = false;
  
  // Second check the IMU change variance
  bool var_stationary = false;
  double var = 0;
  Vector3d aver_a = sum_a * 1.0 / ((int)imu_data.size());
  for (const ov_core::ImuData &data : imu_data){
    var += (data.am - aver_a).dot(data.am - aver_a);
  }
  var = sqrt(var / ((int)imu_data.size() - 1 ));

  if (var < stationary_max_variance)
  {
    var_stationary = true;
  }
  else{
    var_stationary = false;
  }

  // if both norm and various samller than the threshold, imu stationary pass
  if (var_stationary && preint_stationary)
    imu_stationary = true;
  else
    imu_stationary = false;

  return imu_stationary;
}

bool Dynamic_Initializer::static_initialization(Eigen::Matrix<double, 17, 1> &imustate, std::vector<ov_core::ImuData> &imu_data){
  // Return if we don't have any measurements
  if (imu_data.size() < 2) {
    return false;
  }

  // Newest and oldest imu timestamp
  double newesttime = imu_data.back().timestamp;
  double oldesttime = imu_data.front().timestamp;

  if (newesttime - oldesttime < state->op->init->window_time) {
    PRINT0(YELLOW "[INIT-IMU]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // Calculate the sample variance for the newest window from 1 to 0
  Vector3d a_avg = Vector3d::Zero();
  Vector3d w_avg = Vector3d::Zero();
  for (const ImuData &data : imu_data) {
    a_avg += data.am;
    w_avg += data.wm;
  }
  a_avg /= (int)imu_data.size();
  w_avg /= (int)imu_data.size();

  // calculate the various
  double a_var = 0;
  for (const ImuData &data : imu_data) {
    a_var += (data.am - a_avg).dot(data.am - a_avg);
  }
  a_var = std::sqrt(a_var / ((int)imu_data.size() - 1));

  // Get z axis, which aligns with -g (z_in_G=0,0,1)
  Vector3d z_axis = a_avg / a_avg.norm();

  // Create an x_axis
  Vector3d e_1(1, 0, 0);

  // Make x_axis perpendicular to z
  Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  // Get y from the cross product of these two
  Vector3d y_axis = skew_x(z_axis) * x_axis;

  // From these axes get rotation
  Matrix3d Ro;
  Ro.block(0, 0, 3, 1) = x_axis;
  Ro.block(0, 1, 3, 1) = y_axis;
  Ro.block(0, 2, 3, 1) = z_axis;

  // Create our state variables
  Matrix<double, 4, 1> q_GtoI = rot_2_quat(Ro);

  // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
  Vector3d bg = w_avg;
  Vector3d ba = a_avg - quat_2_Rot(q_GtoI) * state->op->gravity;

  // Set our state variables
  imustate(0) = imu_data.at(imu_data.size() - 1).timestamp;
  imustate.block(1, 0, 4, 1) = q_GtoI;
  imustate.block(5, 0, 3, 1) = Vector3d::Zero();
  imustate.block(8, 0, 3, 1) = Vector3d::Zero();
  imustate.block(11, 0, 3, 1) = bg;
  imustate.block(14, 0, 3, 1) = ba;  

  // Return
  PRINT0(GREEN "[Init Debug] Static IMU initialization\n" RESET);
  return true;  
}

bool Dynamic_Initializer::dynamic_initialization(Eigen::Matrix<double, 17, 1> &imustate, std::vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data){
  if (imu_data.size() < 2 || wheel_data.size() < 2){
    PRINT1(YELLOW "[Dyna-Init]:Not eough wheel and imu data.\n" RESET);
    return false;
  }
  
  // Init bg
  Vector3d bg;
  int cnt = 0;
  for (auto wheel : wheel_data) {
    // get the interpolated IMU measurement at the wheel measurement time.
    ImuData imu1, imu2;
    if (!imu_pp->get_bounding_data(wheel.first + toff, imu_data, imu1, imu2))
      continue;
    ImuData imu_itpl = imu_pp->interpolate_data(imu1, imu2, wheel.first + toff);

    // get bg
    Vector3d w_OinO = wheel.second.block(0, 0, 3, 1);
    bg += imu_itpl.wm - R_OtoI * w_OinO;
    cnt++;
  }
  bg /= cnt;

  // Init vi
  Vector3d w_O0inO0 = wheel_data.at(0).second.block(0, 0, 3, 1);
  Vector3d v_O0inO0 = wheel_data.at(0).second.block(3, 0, 3, 1);
  Vector3d v_I0inI0 = R_OtoI * (v_O0inO0 + skew_x(w_O0inO0) * p_IinO);
  
  // Init {I0}^g
  Vector3d g_inI0;
  if (!init_gI_dongsi(bg, v_I0inI0, imu_data, wheel_data, g_inI0))
    return false;

  // Init ba
  Vector3d ba = init_ba(bg, v_I0inI0, g_inI0, imu_data, wheel_data);

  if (ba.norm() > state->op->gravity.norm())
    return false;

  // check residual
  VectorXd res = residual(bg, ba, v_I0inI0, g_inI0, imu_data, wheel_data);


  if (res.tail(3).norm() / 3 > state->op->init->imu_wheel_thresh * 100) {
    PRINT1(RED "[IW-Init]:dynamic_initialization: residual %.2f > %.2f\n" RESET, res.tail(3).norm() / 3, state->op->init->imu_wheel_thresh * 100);
    return false;
  }
  PRINT1(GREEN "[IW-Init]:dynamic_initialization: residual %.2f < %.2f\n" RESET, res.tail(3).norm() / 3, state->op->init->imu_wheel_thresh * 100);

  // init = VectorXd::Zero(12);
  // init.block(0, 0, 3, 1) = bg;
  // init.block(3, 0, 3, 1) = ba;
  // init.block(6, 0, 3, 1) = g_inI0;
  // init.block(9, 0, 3, 1) = v_I0inI0;

  // Init R_GtoI0 which is gravity aligned orientation
  Matrix3d R_GtoI0 = gram_schmidt(g_inI0);

  // Set our state variables
  imustate(0) = imu_data.at(imu_data.size() - 1).timestamp;
  imustate.block(1, 0, 4, 1) = rot_2_quat(R_GtoI0);;
  imustate.block(5, 0, 3, 1) = Vector3d::Zero();
  imustate.block(8, 0, 3, 1) = R_GtoI0.transpose() * v_I0inI0;
  imustate.block(11, 0, 3, 1) = bg;
  imustate.block(14, 0, 3, 1) = ba;  
  return true;
}

bool Dynamic_Initializer::init_gI_dongsi(Vector3d bg, Vector3d v_I0inI0, vector<ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data, Vector3d &gravity_inI0) {

  if (state->op->init->imu_gravity_aligned) {
    gravity_inI0 = state->op->gravity;
    return true;
  }
  //==============================================================================
  // Init {I0}^g and R_GtoI0
  //==============================================================================
  // Assume ba is zero and find the {I0}^g direction.
  // IMU side: V_IkinI0 = V_I0inI0 + Sum(R_IitoI0 * a_inIi * dti) - Sum(dti) * {I0}^g
  // -> {I0}^g = (V_I0inI0 + Sum(R_IitoI0 * a_inIi * dti) - V_I1inI0) / Sum(dti)
  // Wheel side: V_IkinI0 = R_IktoI0 * R_OtoI * ({I0}^v + skew({I0}^w) * p_IinO)

  double sum_dt = 0;
  Vector3d sum_R_a_dt = Vector3d::Zero();
  Matrix3d sum_R_dt = Matrix3d::Zero();
  Matrix3d R_IktoI0 = Matrix3d::Identity();
  Matrix3d R_O0toOk = Matrix3d::Identity();

  MatrixXd A = MatrixXd::Zero(3 * wheel_data.size(), 6);
  VectorXd b = VectorXd::Zero(3 * wheel_data.size());
  for (int i = 1; i < (int)wheel_data.size(); i++) {
    double t_s = wheel_data.at(i - 1).first + toff; // integration start time
    double t_e = wheel_data.at(i).first + toff;     // integration end time
    Vector3d w_Os = wheel_data.at(i - 1).second.block(0, 0, 3, 1);
    Vector3d w_Oe = wheel_data.at(i).second.block(0, 0, 3, 1);
    Vector3d v_Oe = wheel_data.at(i).second.block(3, 0, 3, 1);

    // select imu measurements
    vector<ImuData> imu_prop;
    bool success = imu_pp->select_imu_readings(t_s, t_e, imu_prop);
    assert(success);

    // perform IMU propagation
    for (size_t j = 0; j < imu_prop.size() - 1; j++) {
      // Get IMU measurements
      double dt = imu_prop[j + 1].timestamp - imu_prop[j].timestamp;
      Vector3d w_I0 = imu_prop[j].wm - bg;
      Vector3d w_I1 = imu_prop[j + 1].wm - bg;
      Vector3d a_I = .5 * (imu_prop[j].am + imu_prop[j + 1].am);

      // compute linear system
      sum_R_a_dt += R_IktoI0 * a_I * dt;
      sum_R_dt += R_IktoI0 * dt;
      sum_dt += dt;

      // IMU orientation propagation
      Vector4d q_I0toI1;
      IMU_prop_rk4(dt, w_I0, w_I1, q_I0toI1);
      R_IktoI0 = R_IktoI0.eval() * quat_2_Rot(q_I0toI1).transpose();
    }

    // Wheel orientation propagation
    Vector4d q_O0toO1;
    IMU_prop_rk4(t_e - t_s, w_Os, w_Oe, q_O0toO1);
    R_O0toOk = quat_2_Rot(q_O0toO1) * R_O0toOk.eval();

    // v_ItinI0 created by wheel measurement
    Vector3d v_ItinI0 = R_OtoI * R_O0toOk.transpose() * (v_Oe + skew_x(w_Oe) * p_IinO);

    // Append to the Ax = b system
    b.block(i * 3, 0, 3, 1) = v_ItinI0 - v_I0inI0 - sum_R_a_dt;
    A.block(i * 3, 0, 3, 3) = -sum_R_dt;
    A.block(i * 3, 3, 3, 3) = -sum_dt * Matrix3d::Identity();
  }

  // Constrained solving |g| = 9.81 constraint
  Eigen::MatrixXd A1 = A.block(0, 0, A.rows(), A.cols() - 3);
  // Eigen::MatrixXd A1A1_inv = (A1.transpose() * A1).inverse();
  Eigen::MatrixXd A1A1_inv = (A1.transpose() * A1).llt().solve(Eigen::MatrixXd::Identity(A1.cols(), A1.cols()));
  Eigen::MatrixXd A2 = A.block(0, A.cols() - 3, A.rows(), 3);
  Eigen::MatrixXd Temp = A2.transpose() * (Eigen::MatrixXd::Identity(A1.rows(), A1.rows()) - A1 * A1A1_inv * A1.transpose());
  Eigen::MatrixXd D = Temp * A2;
  Eigen::MatrixXd d = Temp * b;
  Eigen::VectorXd coeff = compute_dongsi_coeff(D, d, state->op->gravity.norm());

  // Get statistics of this problem
  Eigen::JacobiSVD<Eigen::MatrixXd> svd1((A1.transpose() * A1), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd singularValues1 = svd1.singularValues();
  double cond1 = singularValues1(0) / singularValues1(singularValues1.rows() - 1);
  PRINT1("[IW-Init]: A1A1 cond = %.3f | rank = %d of %d (%4.3e threshold)\n", cond1, (int)svd1.rank(), (int)A1.cols(), svd1.threshold());

  // Create companion matrix of our polynomial
  // https://en.wikipedia.org/wiki/Companion_matrix
  assert(coeff(0) == 1);
  Eigen::MatrixXd companion_matrix = Eigen::MatrixXd::Zero(coeff.rows() - 1, coeff.rows() - 1);
  companion_matrix.diagonal(-1).setOnes();
  companion_matrix.col(companion_matrix.cols() - 1) = -coeff.reverse().head(coeff.rows() - 1);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd0(companion_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd singularValues0 = svd0.singularValues();
  double cond0 = singularValues0(0) / singularValues0(singularValues0.rows() - 1);
  PRINT1("[IW-Init]: CM cond = %.3f | rank = %d of %d (%4.3e threshold)\n", cond0, (int)svd0.rank(), (int)companion_matrix.cols(), svd0.threshold());

  // Find its eigenvalues (can be complex)
  Eigen::EigenSolver<Eigen::MatrixXd> solver(companion_matrix, false);
  if (solver.info() != Eigen::Success) {
    PRINT1(RED "[IW-Init]: failed to compute the eigenvalue decomposition!!\n", RESET);
    return false;
  }

  // Find the smallest real eigenvalue
  // NOTE: we find the one that gives us minimal constraint cost
  // NOTE: not sure if the best, but one that gives the correct mag should be good?
  bool lambda_found = false;
  double lambda_min = -1;
  double cost_min = INFINITY;
  Eigen::MatrixXd I_dd = Eigen::MatrixXd::Identity(D.rows(), D.rows());
  // double g2 = params.gravity_mag * params.gravity_mag;
  // Eigen::MatrixXd ddt = d * d.transpose();
  for (int i = 0; i < solver.eigenvalues().size(); i++) {
    auto val = solver.eigenvalues()(i);
    if (val.imag() == 0) {
      double lambda = val.real();
      // Eigen::MatrixXd mat = (D - lambda * I_dd) * (D - lambda * I_dd) - 1 / g2 * ddt;
      // double cost = mat.determinant();
      Eigen::MatrixXd D_lambdaI_inv = (D - lambda * I_dd).llt().solve(I_dd);
      Eigen::VectorXd state_grav = D_lambdaI_inv * d;
      double cost = abs(state_grav.norm() - state->op->gravity.norm());
      if (!lambda_found || cost < cost_min) {
        lambda_found = true;
        lambda_min = lambda;
        cost_min = cost;
      }
    }
  }
  if (!lambda_found) {
    PRINT1(RED "[IW-Init]: failed to find a real eigenvalue!!!\n", RESET);
    return false;
  }

  // Get statistics of this problem
  Eigen::JacobiSVD<Eigen::MatrixXd> svd2((D - lambda_min * I_dd), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd singularValues2 = svd2.singularValues();
  double cond2 = singularValues2(0) / singularValues2(singularValues2.rows() - 1);
  PRINT1("[IW-Init]: (D-lI) cond = %.3f | rank = %d of %d (%4.3e threshold)\n", cond2, (int)svd2.rank(), (int)D.rows(), svd2.threshold());
  PRINT1("[IW-Init]: smallest real eigenvalue = %.5f (cost of %f)\n", lambda_min, cost_min);

  // Recover our gravity from the constraint!
  // Eigen::MatrixXd D_lambdaI_inv = (D - lambda_min * I_dd).inverse();
  Eigen::MatrixXd D_lambdaI_inv = (D - lambda_min * I_dd).llt().solve(I_dd);
  Eigen::VectorXd state_grav = D_lambdaI_inv * d;

  // Check gravity magnitude to see if converged
  gravity_inI0 = state_grav;
  double init_max_grav_difference = 1e-3;
  if (abs(gravity_inI0.norm() - state->op->gravity.norm()) > init_max_grav_difference) {
    PRINT1(YELLOW "[IW-Init]: gravity did not converge  %.3f,%.3f,%.3f (%.3f > %.3f)\n" RESET, gravity_inI0(0), gravity_inI0(1), gravity_inI0(2),
           abs(gravity_inI0.norm() - state->op->gravity.norm()), init_max_grav_difference);
    return false;
  }
  PRINT1("[IW-Init]: gravity in I0 was %.3f,%.3f,%.3f and |g| = %.4f\n", gravity_inI0(0), gravity_inI0(1), gravity_inI0(2), gravity_inI0.norm());
  return true;
}

Vector3d Dynamic_Initializer::init_ba(Vector3d bg, Vector3d v_I0inI0, Vector3d g_inI0, vector<ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data) {

  //==============================================================================
  // Init ba
  //==============================================================================

  Vector3d sum_R_a_dt = Vector3d::Zero();
  Matrix3d sum_R_dt = Matrix3d::Zero();
  double sum_dt = 0;
  Matrix3d R_IktoI0 = Matrix3d::Identity();
  Matrix3d R_O0toOk = Matrix3d::Identity();
  Vector3d ba = Vector3d::Zero();
  for (int i = 1; i < (int)wheel_data.size(); i++) {
    double t_s = wheel_data.at(i - 1).first + toff; // integration start time
    double t_e = wheel_data.at(i).first + toff;     // integration end time
    Vector3d w_Os = wheel_data.at(i - 1).second.block(0, 0, 3, 1);
    Vector3d w_Oe = wheel_data.at(i).second.block(0, 0, 3, 1);
    Vector3d v_Oe = wheel_data.at(i).second.block(3, 0, 3, 1);

    // select imu measurements
    vector<ImuData> imu_prop;
    bool success = imu_pp->select_imu_readings(t_s, t_e, imu_prop);
    assert(success);

    // perform IMU propagation
    for (size_t j = 0; j < imu_prop.size() - 1; j++) {
      // Get IMU measurements
      double dt = imu_prop[j + 1].timestamp - imu_prop[j].timestamp;
      Vector3d w_I0 = imu_prop[j].wm - bg;
      Vector3d w_I1 = imu_prop[j + 1].wm - bg;
      Vector3d a_I = .5 * (imu_prop[j].am + imu_prop[j + 1].am);

      // compute linear system
      sum_R_a_dt += R_IktoI0 * a_I * dt;
      sum_R_dt += R_IktoI0 * dt;
      sum_dt += dt;

      // IMU orientation propagation
      Vector4d q_I0toI1;
      IMU_prop_rk4(dt, w_I0, w_I1, q_I0toI1);
      R_IktoI0 = R_IktoI0.eval() * quat_2_Rot(q_I0toI1).transpose();
    }

    // Wheel orientation propagation
    Vector4d q_O0toO1;
    IMU_prop_rk4(t_e - t_s, w_Os, w_Oe, q_O0toO1);
    R_O0toOk = quat_2_Rot(q_O0toO1) * R_O0toOk.eval();

    // v_ItinI0 created by wheel measurement
    Vector3d v_ItinI0 = R_OtoI * R_O0toOk.transpose() * (v_Oe + skew_x(w_Oe) * p_IinO);

    // sum bias
    ba += sum_R_dt.inverse() * (v_I0inI0 + sum_R_a_dt - sum_dt * g_inI0 - v_ItinI0);
  }

  // get bias
  ba /= (wheel_data.size() - 1);
  return ba;
}


VectorXd Dynamic_Initializer::residual(Vector3d bg, Vector3d ba, Vector3d v_I0inI0, Vector3d g_inI0, vector<ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data) {

  Vector3d sum_R_a_dt = Vector3d::Zero();
  Matrix3d sum_R_dt = Matrix3d::Zero();
  double sum_dt = 0;
  Matrix3d R_IktoI0 = Matrix3d::Identity();
  Matrix3d R_O0toOk = Matrix3d::Identity();

  VectorXd res = VectorXd::Zero(3 * wheel_data.size());
  for (int i = 1; i < (int)wheel_data.size(); i++) {
    double t_s = wheel_data.at(i - 1).first + toff; // integration start time
    double t_e = wheel_data.at(i).first + toff;     // integration end time
    Vector3d w_Os = wheel_data.at(i - 1).second.block(0, 0, 3, 1);
    Vector3d w_Oe = wheel_data.at(i).second.block(0, 0, 3, 1);
    Vector3d v_Oe = wheel_data.at(i).second.block(3, 0, 3, 1);

    // select imu measurements
    vector<ImuData> imu_prop;
    bool success = imu_pp->select_imu_readings(t_s, t_e, imu_prop);
    assert(success);

    // perform IMU propagation
    for (size_t j = 0; j < imu_prop.size() - 1; j++) {
      // Get IMU measurements
      double dt = imu_prop[j + 1].timestamp - imu_prop[j].timestamp;
      Vector3d w_I0 = imu_prop[j].wm - bg;
      Vector3d w_I1 = imu_prop[j + 1].wm - bg;
      Vector3d a_I = .5 * (imu_prop[j].am + imu_prop[j + 1].am);

      // compute linear system
      sum_R_a_dt += R_IktoI0 * a_I * dt;
      sum_R_dt += R_IktoI0 * dt;
      sum_dt += dt;

      // IMU orientation propagation
      Vector4d q_I0toI1;
      IMU_prop_rk4(dt, w_I0, w_I1, q_I0toI1);
      R_IktoI0 = R_IktoI0.eval() * quat_2_Rot(q_I0toI1).transpose();
    }

    // Wheel orientation propagation
    Vector4d q_O0toO1;
    IMU_prop_rk4(t_e - t_s, w_Os, w_Oe, q_O0toO1);
    R_O0toOk = quat_2_Rot(q_O0toO1) * R_O0toOk.eval();

    // v_ItinI0 created by wheel measurement
    Vector3d v_ItinI0 = R_OtoI * R_O0toOk.transpose() * (v_Oe + skew_x(w_Oe) * p_IinO);

    // sum res
    res.block(i * 3, 0, 3, 1) = v_ItinI0 - v_I0inI0 - sum_R_a_dt + sum_R_dt * ba + sum_dt * g_inI0;
  }
  return res;
}

void Dynamic_Initializer::IMU_prop_rk4(double dt, const Vector3d &w_hat1, const Vector3d &w_hat2, Vector4d &new_q) {

  // Pre-compute things
  Vector3d w_hat = w_hat1;
  Vector3d w_alpha = (w_hat2 - w_hat1) / dt;

  // k1 ================
  Vector4d dq_0 = {0, 0, 0, 1};
  Vector4d q0_dot = 0.5 * Omega(w_hat) * dq_0;
  Vector4d k1_q = q0_dot * dt;

  // k2 ================
  w_hat += 0.5 * w_alpha * dt;
  Vector4d dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
  Vector4d q1_dot = 0.5 * Omega(w_hat) * dq_1;
  Vector4d k2_q = q1_dot * dt;

  // k3 ================
  Vector4d dq_2 = quatnorm(dq_0 + 0.5 * k2_q);
  Vector4d q2_dot = 0.5 * Omega(w_hat) * dq_2;
  Vector4d k3_q = q2_dot * dt;

  // k4 ================
  w_hat += 0.5 * w_alpha * dt;
  Vector4d dq_3 = quatnorm(dq_0 + k3_q);
  Vector4d q3_dot = 0.5 * Omega(w_hat) * dq_3;
  Vector4d k4_q = q3_dot * dt;

  // y+dt ================
  new_q = quatnorm(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q);
}

Matrix3d Dynamic_Initializer::gram_schmidt(const Vector3d &gravity_inI) {

  // Now gram schmidt to find rot for grav
  // https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
  // Get z axis, which alines with -g (z_in_G=0,0,1)
  Vector3d z_axis = gravity_inI / gravity_inI.norm();
  Vector3d x_axis, y_axis;
  Vector3d e_1(1.0, 0.0, 0.0);

  // Original method
  x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();
  y_axis = ov_core::skew_x(z_axis) * x_axis;
  y_axis = y_axis / y_axis.norm();

  // Rotation from our global (where gravity is only along the z-axis) to the local one
  Matrix3d R_GtoI;
  R_GtoI.block(0, 0, 3, 1) = x_axis;
  R_GtoI.block(0, 1, 3, 1) = y_axis;
  R_GtoI.block(0, 2, 3, 1) = z_axis;
  return R_GtoI;
}

VectorXd Dynamic_Initializer::compute_dongsi_coeff(MatrixXd &D, const MatrixXd &d, double gravity_mag) {

  // matlab constants
  assert(D.rows() == 3);
  assert(D.cols() == 3);
  assert(d.rows() == 3);
  double D1_1 = D(0, 0), D1_2 = D(0, 1), D1_3 = D(0, 2);
  double D2_1 = D(1, 0), D2_2 = D(1, 1), D2_3 = D(1, 2);
  double D3_1 = D(2, 0), D3_2 = D(2, 1), D3_3 = D(2, 2);
  double d1 = d(0, 0), d2 = d(1, 0), d3 = d(2, 0);
  double g = gravity_mag;

  // squared version we subbed for x^2
  double D1_1_sq = D1_1 * D1_1, D1_2_sq = D1_2 * D1_2, D1_3_sq = D1_3 * D1_3;
  double D2_1_sq = D2_1 * D2_1, D2_2_sq = D2_2 * D2_2, D2_3_sq = D2_3 * D2_3;
  double D3_1_sq = D3_1 * D3_1, D3_2_sq = D3_2 * D3_2, D3_3_sq = D3_3 * D3_3;
  double d1_sq = d1 * d1, d2_sq = d2 * d2, d3_sq = d3 * d3;
  double g_sq = g * g;

  // Compute the coefficients
  VectorXd coeff = VectorXd::Zero(7);
  coeff(6) =
      -(-D1_1_sq * D2_2_sq * D3_3_sq * g_sq + D1_1_sq * D2_2_sq * d3_sq + 2 * D1_1_sq * D2_2 * D2_3 * D3_2 * D3_3 * g_sq - D1_1_sq * D2_2 * D2_3 * d2 * d3 -
        D1_1_sq * D2_2 * D3_2 * d2 * d3 - D1_1_sq * D2_3_sq * D3_2_sq * g_sq + D1_1_sq * D2_3 * D3_2 * d2_sq + D1_1_sq * D2_3 * D3_2 * d3_sq - D1_1_sq * D2_3 * D3_3 * d2 * d3 -
        D1_1_sq * D3_2 * D3_3 * d2 * d3 + D1_1_sq * D3_3_sq * d2_sq + 2 * D1_1 * D1_2 * D2_1 * D2_2 * D3_3_sq * g_sq - 2 * D1_1 * D1_2 * D2_1 * D2_2 * d3_sq -
        2 * D1_1 * D1_2 * D2_1 * D2_3 * D3_2 * D3_3 * g_sq + D1_1 * D1_2 * D2_1 * D2_3 * d2 * d3 + D1_1 * D1_2 * D2_1 * D3_2 * d2 * d3 -
        2 * D1_1 * D1_2 * D2_2 * D2_3 * D3_1 * D3_3 * g_sq + D1_1 * D1_2 * D2_2 * D2_3 * d1 * d3 + D1_1 * D1_2 * D2_2 * D3_1 * d2 * d3 +
        2 * D1_1 * D1_2 * D2_3_sq * D3_1 * D3_2 * g_sq - D1_1 * D1_2 * D2_3 * D3_1 * d2_sq - D1_1 * D1_2 * D2_3 * D3_1 * d3_sq - D1_1 * D1_2 * D2_3 * D3_2 * d1 * d2 +
        D1_1 * D1_2 * D2_3 * D3_3 * d1 * d3 + D1_1 * D1_2 * D3_1 * D3_3 * d2 * d3 - D1_1 * D1_2 * D3_3_sq * d1 * d2 - 2 * D1_1 * D1_3 * D2_1 * D2_2 * D3_2 * D3_3 * g_sq +
        D1_1 * D1_3 * D2_1 * D2_2 * d2 * d3 + 2 * D1_1 * D1_3 * D2_1 * D2_3 * D3_2_sq * g_sq - D1_1 * D1_3 * D2_1 * D3_2 * d2_sq - D1_1 * D1_3 * D2_1 * D3_2 * d3_sq +
        D1_1 * D1_3 * D2_1 * D3_3 * d2 * d3 + 2 * D1_1 * D1_3 * D2_2_sq * D3_1 * D3_3 * g_sq - D1_1 * D1_3 * D2_2_sq * d1 * d3 -
        2 * D1_1 * D1_3 * D2_2 * D2_3 * D3_1 * D3_2 * g_sq + D1_1 * D1_3 * D2_2 * D3_2 * d1 * d2 + D1_1 * D1_3 * D2_3 * D3_1 * d2 * d3 - D1_1 * D1_3 * D2_3 * D3_2 * d1 * d3 +
        D1_1 * D1_3 * D3_1 * D3_2 * d2 * d3 - 2 * D1_1 * D1_3 * D3_1 * D3_3 * d2_sq + D1_1 * D1_3 * D3_2 * D3_3 * d1 * d2 + D1_1 * D2_1 * D2_2 * D3_2 * d1 * d3 -
        D1_1 * D2_1 * D2_3 * D3_2 * d1 * d2 + D1_1 * D2_1 * D3_2 * D3_3 * d1 * d3 - D1_1 * D2_1 * D3_3_sq * d1 * d2 - D1_1 * D2_2_sq * D3_1 * d1 * d3 +
        D1_1 * D2_2 * D2_3 * D3_1 * d1 * d2 - D1_1 * D2_3 * D3_1 * D3_2 * d1 * d3 + D1_1 * D2_3 * D3_1 * D3_3 * d1 * d2 - D1_2_sq * D2_1_sq * D3_3_sq * g_sq +
        D1_2_sq * D2_1_sq * d3_sq + 2 * D1_2_sq * D2_1 * D2_3 * D3_1 * D3_3 * g_sq - D1_2_sq * D2_1 * D2_3 * d1 * d3 - D1_2_sq * D2_1 * D3_1 * d2 * d3 -
        D1_2_sq * D2_3_sq * D3_1_sq * g_sq + D1_2_sq * D2_3 * D3_1 * d1 * d2 + 2 * D1_2 * D1_3 * D2_1_sq * D3_2 * D3_3 * g_sq - D1_2 * D1_3 * D2_1_sq * d2 * d3 -
        2 * D1_2 * D1_3 * D2_1 * D2_2 * D3_1 * D3_3 * g_sq + D1_2 * D1_3 * D2_1 * D2_2 * d1 * d3 - 2 * D1_2 * D1_3 * D2_1 * D2_3 * D3_1 * D3_2 * g_sq +
        D1_2 * D1_3 * D2_1 * D3_1 * d2_sq + D1_2 * D1_3 * D2_1 * D3_1 * d3_sq - D1_2 * D1_3 * D2_1 * D3_3 * d1 * d3 + 2 * D1_2 * D1_3 * D2_2 * D2_3 * D3_1_sq * g_sq -
        D1_2 * D1_3 * D2_2 * D3_1 * d1 * d2 - D1_2 * D1_3 * D3_1_sq * d2 * d3 + D1_2 * D1_3 * D3_1 * D3_3 * d1 * d2 - D1_2 * D2_1_sq * D3_2 * d1 * d3 +
        D1_2 * D2_1 * D2_2 * D3_1 * d1 * d3 + D1_2 * D2_1 * D2_3 * D3_2 * d1_sq + D1_2 * D2_1 * D2_3 * D3_2 * d3_sq - D1_2 * D2_1 * D2_3 * D3_3 * d2 * d3 -
        D1_2 * D2_1 * D3_1 * D3_3 * d1 * d3 - D1_2 * D2_1 * D3_2 * D3_3 * d2 * d3 + D1_2 * D2_1 * D3_3_sq * d1_sq + D1_2 * D2_1 * D3_3_sq * d2_sq -
        D1_2 * D2_2 * D2_3 * D3_1 * d1_sq - D1_2 * D2_2 * D2_3 * D3_1 * d3_sq + D1_2 * D2_2 * D2_3 * D3_3 * d1 * d3 + D1_2 * D2_2 * D3_1 * D3_3 * d2 * d3 -
        D1_2 * D2_2 * D3_3_sq * d1 * d2 + D1_2 * D2_3_sq * D3_1 * d2 * d3 - D1_2 * D2_3_sq * D3_2 * d1 * d3 + D1_2 * D2_3 * D3_1_sq * d1 * d3 - D1_2 * D2_3 * D3_1 * D3_3 * d1_sq -
        D1_2 * D2_3 * D3_1 * D3_3 * d2_sq + D1_2 * D2_3 * D3_2 * D3_3 * d1 * d2 - D1_3_sq * D2_1_sq * D3_2_sq * g_sq + 2 * D1_3_sq * D2_1 * D2_2 * D3_1 * D3_2 * g_sq -
        D1_3_sq * D2_1 * D3_1 * d2 * d3 + D1_3_sq * D2_1 * D3_2 * d1 * d3 - D1_3_sq * D2_2_sq * D3_1_sq * g_sq + D1_3_sq * D3_1_sq * d2_sq - D1_3_sq * D3_1 * D3_2 * d1 * d2 +
        D1_3 * D2_1_sq * D3_2 * d1 * d2 - D1_3 * D2_1 * D2_2 * D3_1 * d1 * d2 - D1_3 * D2_1 * D2_2 * D3_2 * d1_sq - D1_3 * D2_1 * D2_2 * D3_2 * d3_sq +
        D1_3 * D2_1 * D2_2 * D3_3 * d2 * d3 + D1_3 * D2_1 * D3_1 * D3_3 * d1 * d2 + D1_3 * D2_1 * D3_2_sq * d2 * d3 - D1_3 * D2_1 * D3_2 * D3_3 * d1_sq -
        D1_3 * D2_1 * D3_2 * D3_3 * d2_sq + D1_3 * D2_2_sq * D3_1 * d1_sq + D1_3 * D2_2_sq * D3_1 * d3_sq - D1_3 * D2_2_sq * D3_3 * d1 * d3 - D1_3 * D2_2 * D2_3 * D3_1 * d2 * d3 +
        D1_3 * D2_2 * D2_3 * D3_2 * d1 * d3 - D1_3 * D2_2 * D3_1 * D3_2 * d2 * d3 + D1_3 * D2_2 * D3_2 * D3_3 * d1 * d2 - D1_3 * D2_3 * D3_1_sq * d1 * d2 +
        D1_3 * D2_3 * D3_1 * D3_2 * d1_sq + D1_3 * D2_3 * D3_1 * D3_2 * d2_sq - D1_3 * D2_3 * D3_2_sq * d1 * d2 + D2_1 * D2_2 * D3_2 * D3_3 * d1 * d3 -
        D2_1 * D2_2 * D3_3_sq * d1 * d2 - D2_1 * D2_3 * D3_2_sq * d1 * d3 + D2_1 * D2_3 * D3_2 * D3_3 * d1 * d2 - D2_2_sq * D3_1 * D3_3 * d1 * d3 + D2_2_sq * D3_3_sq * d1_sq +
        D2_2 * D2_3 * D3_1 * D3_2 * d1 * d3 + D2_2 * D2_3 * D3_1 * D3_3 * d1 * d2 - 2 * D2_2 * D2_3 * D3_2 * D3_3 * d1_sq - D2_3_sq * D3_1 * D3_2 * d1 * d2 +
        D2_3_sq * D3_2_sq * d1_sq) /
      g_sq;
  coeff(5) =
      (-(2 * D1_1_sq * D2_2_sq * D3_3 * g_sq - 2 * D1_1_sq * D2_2 * D2_3 * D3_2 * g_sq + 2 * D1_1_sq * D2_2 * D3_3_sq * g_sq - 2 * D1_1_sq * D2_2 * d3_sq -
         2 * D1_1_sq * D2_3 * D3_2 * D3_3 * g_sq + 2 * D1_1_sq * D2_3 * d2 * d3 + 2 * D1_1_sq * D3_2 * d2 * d3 - 2 * D1_1_sq * D3_3 * d2_sq -
         4 * D1_1 * D1_2 * D2_1 * D2_2 * D3_3 * g_sq + 2 * D1_1 * D1_2 * D2_1 * D2_3 * D3_2 * g_sq - 2 * D1_1 * D1_2 * D2_1 * D3_3_sq * g_sq + 2 * D1_1 * D1_2 * D2_1 * d3_sq +
         2 * D1_1 * D1_2 * D2_2 * D2_3 * D3_1 * g_sq + 2 * D1_1 * D1_2 * D2_3 * D3_1 * D3_3 * g_sq - 2 * D1_1 * D1_2 * D2_3 * d1 * d3 - 2 * D1_1 * D1_2 * D3_1 * d2 * d3 +
         2 * D1_1 * D1_2 * D3_3 * d1 * d2 + 2 * D1_1 * D1_3 * D2_1 * D2_2 * D3_2 * g_sq + 2 * D1_1 * D1_3 * D2_1 * D3_2 * D3_3 * g_sq - 2 * D1_1 * D1_3 * D2_1 * d2 * d3 -
         2 * D1_1 * D1_3 * D2_2_sq * D3_1 * g_sq - 4 * D1_1 * D1_3 * D2_2 * D3_1 * D3_3 * g_sq + 2 * D1_1 * D1_3 * D2_2 * d1 * d3 + 2 * D1_1 * D1_3 * D2_3 * D3_1 * D3_2 * g_sq +
         2 * D1_1 * D1_3 * D3_1 * d2_sq - 2 * D1_1 * D1_3 * D3_2 * d1 * d2 - 2 * D1_1 * D2_1 * D3_2 * d1 * d3 + 2 * D1_1 * D2_1 * D3_3 * d1 * d2 +
         2 * D1_1 * D2_2_sq * D3_3_sq * g_sq - 2 * D1_1 * D2_2_sq * d3_sq - 4 * D1_1 * D2_2 * D2_3 * D3_2 * D3_3 * g_sq + 2 * D1_1 * D2_2 * D2_3 * d2 * d3 +
         2 * D1_1 * D2_2 * D3_1 * d1 * d3 + 2 * D1_1 * D2_2 * D3_2 * d2 * d3 + 2 * D1_1 * D2_3_sq * D3_2_sq * g_sq - 2 * D1_1 * D2_3 * D3_1 * d1 * d2 -
         2 * D1_1 * D2_3 * D3_2 * d2_sq - 2 * D1_1 * D2_3 * D3_2 * d3_sq + 2 * D1_1 * D2_3 * D3_3 * d2 * d3 + 2 * D1_1 * D3_2 * D3_3 * d2 * d3 - 2 * D1_1 * D3_3_sq * d2_sq +
         2 * D1_2_sq * D2_1_sq * D3_3 * g_sq - 2 * D1_2_sq * D2_1 * D2_3 * D3_1 * g_sq - 2 * D1_2 * D1_3 * D2_1_sq * D3_2 * g_sq + 2 * D1_2 * D1_3 * D2_1 * D2_2 * D3_1 * g_sq +
         2 * D1_2 * D1_3 * D2_1 * D3_1 * D3_3 * g_sq - 2 * D1_2 * D1_3 * D2_3 * D3_1_sq * g_sq - 2 * D1_2 * D2_1 * D2_2 * D3_3_sq * g_sq + 2 * D1_2 * D2_1 * D2_2 * d3_sq +
         2 * D1_2 * D2_1 * D2_3 * D3_2 * D3_3 * g_sq - 2 * D1_2 * D2_1 * D3_3 * d1_sq - 2 * D1_2 * D2_1 * D3_3 * d2_sq + 2 * D1_2 * D2_2 * D2_3 * D3_1 * D3_3 * g_sq -
         2 * D1_2 * D2_2 * D2_3 * d1 * d3 - 2 * D1_2 * D2_2 * D3_1 * d2 * d3 + 2 * D1_2 * D2_2 * D3_3 * d1 * d2 - 2 * D1_2 * D2_3_sq * D3_1 * D3_2 * g_sq +
         2 * D1_2 * D2_3 * D3_1 * d1_sq + 2 * D1_2 * D2_3 * D3_1 * d2_sq + 2 * D1_2 * D2_3 * D3_1 * d3_sq - 2 * D1_2 * D2_3 * D3_3 * d1 * d3 - 2 * D1_2 * D3_1 * D3_3 * d2 * d3 +
         2 * D1_2 * D3_3_sq * d1 * d2 - 2 * D1_3_sq * D2_1 * D3_1 * D3_2 * g_sq + 2 * D1_3_sq * D2_2 * D3_1_sq * g_sq + 2 * D1_3 * D2_1 * D2_2 * D3_2 * D3_3 * g_sq -
         2 * D1_3 * D2_1 * D2_2 * d2 * d3 - 2 * D1_3 * D2_1 * D2_3 * D3_2_sq * g_sq + 2 * D1_3 * D2_1 * D3_2 * d1_sq + 2 * D1_3 * D2_1 * D3_2 * d2_sq +
         2 * D1_3 * D2_1 * D3_2 * d3_sq - 2 * D1_3 * D2_1 * D3_3 * d2 * d3 - 2 * D1_3 * D2_2_sq * D3_1 * D3_3 * g_sq + 2 * D1_3 * D2_2_sq * d1 * d3 +
         2 * D1_3 * D2_2 * D2_3 * D3_1 * D3_2 * g_sq - 2 * D1_3 * D2_2 * D3_1 * d1_sq - 2 * D1_3 * D2_2 * D3_1 * d3_sq - 2 * D1_3 * D2_2 * D3_2 * d1 * d2 +
         2 * D1_3 * D2_2 * D3_3 * d1 * d3 + 2 * D1_3 * D3_1 * D3_3 * d2_sq - 2 * D1_3 * D3_2 * D3_3 * d1 * d2 - 2 * D2_1 * D2_2 * D3_2 * d1 * d3 +
         2 * D2_1 * D2_2 * D3_3 * d1 * d2 - 2 * D2_1 * D3_2 * D3_3 * d1 * d3 + 2 * D2_1 * D3_3_sq * d1 * d2 + 2 * D2_2_sq * D3_1 * d1 * d3 - 2 * D2_2_sq * D3_3 * d1_sq -
         2 * D2_2 * D2_3 * D3_1 * d1 * d2 + 2 * D2_2 * D2_3 * D3_2 * d1_sq + 2 * D2_2 * D3_1 * D3_3 * d1 * d3 - 2 * D2_2 * D3_3_sq * d1_sq - 2 * D2_3 * D3_1 * D3_3 * d1 * d2 +
         2 * D2_3 * D3_2 * D3_3 * d1_sq) /
       g_sq);
  coeff(4) = ((D1_1_sq * D2_2_sq * g_sq + 4 * D1_1_sq * D2_2 * D3_3 * g_sq - 2 * D1_1_sq * D2_3 * D3_2 * g_sq + D1_1_sq * D3_3_sq * g_sq - D1_1_sq * d2_sq - D1_1_sq * d3_sq -
               2 * D1_1 * D1_2 * D2_1 * D2_2 * g_sq - 4 * D1_1 * D1_2 * D2_1 * D3_3 * g_sq + 2 * D1_1 * D1_2 * D2_3 * D3_1 * g_sq + D1_1 * D1_2 * d1 * d2 +
               2 * D1_1 * D1_3 * D2_1 * D3_2 * g_sq - 4 * D1_1 * D1_3 * D2_2 * D3_1 * g_sq - 2 * D1_1 * D1_3 * D3_1 * D3_3 * g_sq + D1_1 * D1_3 * d1 * d3 + D1_1 * D2_1 * d1 * d2 +
               4 * D1_1 * D2_2_sq * D3_3 * g_sq - 4 * D1_1 * D2_2 * D2_3 * D3_2 * g_sq + 4 * D1_1 * D2_2 * D3_3_sq * g_sq - 4 * D1_1 * D2_2 * d3_sq -
               4 * D1_1 * D2_3 * D3_2 * D3_3 * g_sq + 4 * D1_1 * D2_3 * d2 * d3 + D1_1 * D3_1 * d1 * d3 + 4 * D1_1 * D3_2 * d2 * d3 - 4 * D1_1 * D3_3 * d2_sq +
               D1_2_sq * D2_1_sq * g_sq + 2 * D1_2 * D1_3 * D2_1 * D3_1 * g_sq - 4 * D1_2 * D2_1 * D2_2 * D3_3 * g_sq + 2 * D1_2 * D2_1 * D2_3 * D3_2 * g_sq -
               2 * D1_2 * D2_1 * D3_3_sq * g_sq - D1_2 * D2_1 * d1_sq - D1_2 * D2_1 * d2_sq + 2 * D1_2 * D2_1 * d3_sq + 2 * D1_2 * D2_2 * D2_3 * D3_1 * g_sq +
               D1_2 * D2_2 * d1 * d2 + 2 * D1_2 * D2_3 * D3_1 * D3_3 * g_sq - 3 * D1_2 * D2_3 * d1 * d3 - 3 * D1_2 * D3_1 * d2 * d3 + 4 * D1_2 * D3_3 * d1 * d2 +
               D1_3_sq * D3_1_sq * g_sq + 2 * D1_3 * D2_1 * D2_2 * D3_2 * g_sq + 2 * D1_3 * D2_1 * D3_2 * D3_3 * g_sq - 3 * D1_3 * D2_1 * d2 * d3 -
               2 * D1_3 * D2_2_sq * D3_1 * g_sq - 4 * D1_3 * D2_2 * D3_1 * D3_3 * g_sq + 4 * D1_3 * D2_2 * d1 * d3 + 2 * D1_3 * D2_3 * D3_1 * D3_2 * g_sq - D1_3 * D3_1 * d1_sq +
               2 * D1_3 * D3_1 * d2_sq - D1_3 * D3_1 * d3_sq - 3 * D1_3 * D3_2 * d1 * d2 + D1_3 * D3_3 * d1 * d3 + D2_1 * D2_2 * d1 * d2 - 3 * D2_1 * D3_2 * d1 * d3 +
               4 * D2_1 * D3_3 * d1 * d2 + D2_2_sq * D3_3_sq * g_sq - D2_2_sq * d1_sq - D2_2_sq * d3_sq - 2 * D2_2 * D2_3 * D3_2 * D3_3 * g_sq + D2_2 * D2_3 * d2 * d3 +
               4 * D2_2 * D3_1 * d1 * d3 + D2_2 * D3_2 * d2 * d3 - 4 * D2_2 * D3_3 * d1_sq + D2_3_sq * D3_2_sq * g_sq - 3 * D2_3 * D3_1 * d1 * d2 + 2 * D2_3 * D3_2 * d1_sq -
               D2_3 * D3_2 * d2_sq - D2_3 * D3_2 * d3_sq + D2_3 * D3_3 * d2 * d3 + D3_1 * D3_3 * d1 * d3 + D3_2 * D3_3 * d2 * d3 - D3_3_sq * d1_sq - D3_3_sq * d2_sq) /
              g_sq);
  coeff(3) =
      ((2 * D1_1 * d2_sq + 2 * D1_1 * d3_sq + 2 * D2_2 * d1_sq + 2 * D2_2 * d3_sq + 2 * D3_3 * d1_sq + 2 * D3_3 * d2_sq - 2 * D1_1 * D2_2_sq * g_sq - 2 * D1_1_sq * D2_2 * g_sq -
        2 * D1_1 * D3_3_sq * g_sq - 2 * D1_1_sq * D3_3 * g_sq - 2 * D2_2 * D3_3_sq * g_sq - 2 * D2_2_sq * D3_3 * g_sq - 2 * D1_2 * d1 * d2 - 2 * D1_3 * d1 * d3 -
        2 * D2_1 * d1 * d2 - 2 * D2_3 * d2 * d3 - 2 * D3_1 * d1 * d3 - 2 * D3_2 * d2 * d3 + 2 * D1_1 * D1_2 * D2_1 * g_sq + 2 * D1_1 * D1_3 * D3_1 * g_sq +
        2 * D1_2 * D2_1 * D2_2 * g_sq - 8 * D1_1 * D2_2 * D3_3 * g_sq + 4 * D1_1 * D2_3 * D3_2 * g_sq + 4 * D1_2 * D2_1 * D3_3 * g_sq - 2 * D1_2 * D2_3 * D3_1 * g_sq -
        2 * D1_3 * D2_1 * D3_2 * g_sq + 4 * D1_3 * D2_2 * D3_1 * g_sq + 2 * D1_3 * D3_1 * D3_3 * g_sq + 2 * D2_2 * D2_3 * D3_2 * g_sq + 2 * D2_3 * D3_2 * D3_3 * g_sq) /
       g_sq);
  coeff(2) = (-(d1_sq + d2_sq + d3_sq - D1_1_sq * g_sq - D2_2_sq * g_sq - D3_3_sq * g_sq - 4 * D1_1 * D2_2 * g_sq + 2 * D1_2 * D2_1 * g_sq - 4 * D1_1 * D3_3 * g_sq +
                2 * D1_3 * D3_1 * g_sq - 4 * D2_2 * D3_3 * g_sq + 2 * D2_3 * D3_2 * g_sq) /
              g_sq);
  coeff(1) = (-(2 * D1_1 * g_sq + 2 * D2_2 * g_sq + 2 * D3_3 * g_sq) / g_sq);
  coeff(0) = 1;

  // finally return
  return coeff;
}