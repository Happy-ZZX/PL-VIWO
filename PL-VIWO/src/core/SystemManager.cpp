#include "SystemManager.h"
#include "init/Initializer.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsWheel.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "update/UpdaterStatistics.h"
#include "update/cam/CamTypes.h"
#include "update/cam/UpdaterCamera.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/MathGPS.h"
#include "update/gps/UpdaterGPS.h"
#include "update/wheel/UpdaterWheel.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Jabdongsani.h"
#include "utils/TimeChecker.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"

#include <ros/console.h>

using namespace std;
using namespace viw;

SystemManager::SystemManager(shared_ptr<OptionsEstimator> op) {
  // Create "THE MOST IMPORTANT" state
  state = std::make_shared<State>(op);
  tc_sensors = std::make_shared<TimeChecker>();

  // Updaters
  state->op->cam->enabled ? up_cam = make_shared<UpdaterCamera>(state) : shared_ptr<UpdaterCamera>();
  state->op->gps->enabled ? up_gps = make_shared<UpdaterGPS>(state) : shared_ptr<UpdaterGPS>();
  state->op->wheel->enabled ? up_whl = make_shared<UpdaterWheel>(state) : shared_ptr<UpdaterWheel>();

  // Propagator & Initializer
  prop = std::make_shared<Propagator>(state);
  state->op->use_imu_res ? state->hook_propagator(prop) : void();
  initializer = std::make_shared<Initializer>(state, prop, up_whl, up_gps, up_cam);

  // Average interpolation order and cloning frequency
  avg_order = make_shared<STAT>();
  avg_freq = make_shared<STAT>();
  
  //zero-velocity updater
  if (state->op->cam->enabled && state->op->wheel->enabled){
    state->op->zupt->enabled ? up_zupt = make_shared<ZuptUpdater>(state, prop, up_whl, up_cam) : shared_ptr<ZuptUpdater>();
  }
}

bool SystemManager::feed_measurement_imu(const ov_core::ImuData &imu) {
  // Feed our propagator
  prop->feed_imu(imu);

  // Try initialization if not initialized
  if (!state->initialized && !initializer->try_initializtion())
    return false;

  // The following steps should be processed after system initialized
  assert(state->initialized);
  tc_sensors->ding("IMU");

  // erase IMU pose in the clones if exist
  for (auto clone = state->clones.cbegin(); clone != state->clones.cend();)
    clone->second->id() == state->imu->id() ? state->clones.erase(clone++) : clone++;
  // Check if we want to create a new clone
  double clone_time = -1;
  if (get_next_clone_time(clone_time, imu.timestamp)) {
    // Propagate the state to clone time
    prop->propagate(clone_time);
    // Stochastic cloning
    StateHelper::augment_clone(state);
    // Marginalization
    StateHelper::marginalize_old_clone(state);
    // Reset CPI
    prop->reset_cpi(state->time);
    // Reset estimated acceleration
    state->est_A->reset();
    state->est_a->reset();
    // flush old data that state has
    state->flush_old_data();
    // Increase the counter for time
    tc_sensors->counter++;
    // Print the state
    print_status();
  }

  // Propagate the state to the latest IMU time
  prop->propagate(imu.timestamp);

  // Insert IMU pose in the clone list
  if (!state->have_clone(state->time))
    state->clones.insert({state->time, state->imu->pose()});

  // Update polynomial
  state->add_polynomial();
  tc_sensors->dong("IMU");

  // do visualization
  return clone_time > 0;
}

void SystemManager::feed_measurement_camera(const CameraData &cam) {
  // boost::posix_time::ptime rT1, rT2;
  // rT1 = boost::posix_time::microsec_clock::local_time();
  if (!state->op->cam->enabled)
    return;
  // Feed measurement
  state->initialized ? tc_sensors->ding("CAM") : void();
  up_cam->feed_measurement(cam);

  // First check if curret state is stationary
  if (state->initialized){
      state->initialized ? tc_sensors->dong("CAM") : void();
      last_zupt_time = cam.timestamp;
      return;
  }
  // Try update
  state->initialized ? up_cam->try_update(cam.sensor_ids.at(0)) : void();
  state->initialized ? tc_sensors->dong("CAM") : void();
  // rT2 = boost::posix_time::microsec_clock::local_time();
  // cout << "Total process time is " << (rT2 - rT1).total_microseconds() * 1e-3 << endl;
}

void SystemManager::feed_measurement_wheel(const WheelData &wheel) {
  if (!state->op->wheel->enabled)
    return;
  state->initialized ? tc_sensors->ding("WHL") : void();
  // Feed measurement & try update
  up_whl->feed_measurement(wheel);
  state->initialized ? up_whl->try_update() : void();
  state->initialized ? tc_sensors->dong("WHL") : void();
}

void SystemManager::feed_measurement_gps(GPSData gps, bool isGeodetic) {
  if (!state->op->gps->enabled || !state->initialized)
    return;

  if (isnan(gps.noise.norm()) || gps.noise.norm() > 100)
    return;

  state->initialized ? tc_sensors->ding("GPS") : void();
  // If we didn't set the datum yet, take the first gps measurement and use it as the datum
  if (gps_datum.hasNaN()) {
    gps_datum = gps.meas;
    PRINT2(CYAN "[GPS]: Datum set: %.3f,%.3f,%.3f\n" RESET, gps.meas(0), gps.meas(1), gps.meas(2));
  }

  // Convert from a geodetic WGS-84 coordinated to East-North-Up
  gps.meas = isGeodetic ? MathGPS::GeodeticToEnu(gps.meas, gps_datum) : gps.meas;

  // Add keyframe if not initialized.
  !up_gps->initialized ? up_gps->add_keyframes(gps) : void();

  // Feed measurement & try update
  up_gps->feed_measurement(gps);
  up_gps->try_update();

  // Remove keyframes if initialized.
  if (up_gps->initialized && !state->keyframes.empty()) {
    state->keyframes.clear();
    state->keyframes_candidate.clear();
    StateHelper::marginalize_old_clone(state);
  }
  state->initialized ? tc_sensors->dong("GPS") : void();
}

bool SystemManager::get_next_clone_time(double &clone_time, double meas_t) {
  // Return if system not initialized yet.
  if (!state->initialized)
    return false;

  // Create clone RIGHT NOW if we do not have any.
  if (state->clones.empty()) {
    clone_time = state->time;
    return true;
  }

  // Change the clone hz and interpolation order if dynamic cloning is enabled
  compute_accelerations();
  int freq = state->op->clone_freq;
  int intr = state->op->intr_order;
  state->op->dynamic_cloning ? dynamic_cloning(freq, intr) : void();

  // Set desired clone time
  if (state->clones.empty()) // no clone
    clone_time = meas_t;
  else if (state->clones.at(state->newest_clone_time())->id() == state->imu->id()) // last clone is IMU pose
    clone_time = state->newest_2nd_clone_time() + 1.0 / freq;
  else
    clone_time = state->newest_clone_time() + 1.0 / freq;
  // Make sure the desired clone time is not before current state time
  clone_time = clone_time < state->time ? meas_t : clone_time;
  PRINT0("SystemManager::get_next_clone_time:Desired clone time: %.4f ", clone_time);

  // Find the nearest sensor measurement to this clone time (up to sensor timeoffset)
  // only allow cloning -10% < of desired clone time to prevent too dense cloning.
  double min_t_diff = INFINITY;
  double tmp_clone_time = clone_time;
  bool have_meas = false;
  double newest_clone_t = state->newest_clone_time();

  // Cam
  if (state->op->cam->enabled) {
    for (auto meas : up_cam->t_hist) {
      for (auto t = meas.second.rbegin(); t != meas.second.rend(); ++t) {
        double sensor_t = *t + state->cam_dt.at(meas.first)->value()(0);
        if (newest_clone_t < sensor_t)
          have_meas = true;
        if (sensor_t < clone_time - 0.1 / freq)
          break;
        if (abs(sensor_t - clone_time) < min_t_diff && sensor_t >= state->time) {
          min_t_diff = abs(sensor_t - clone_time);
          tmp_clone_time = sensor_t;
        }
      }
    }
  }

  // GNSS
  if (state->op->gps->enabled) {
    for (auto meas : up_gps->t_hist) {
      for (auto t = meas.second.rbegin(); t != meas.second.rend(); ++t) {
        double sensor_t = *t + state->gps_dt.at(meas.first)->value()(0);
        if (newest_clone_t < sensor_t)
          have_meas = true;
        if (sensor_t < clone_time - 0.1 / freq)
          break;
        if (abs(sensor_t - clone_time) < min_t_diff && sensor_t >= state->time) {
          min_t_diff = abs(sensor_t - clone_time);
          tmp_clone_time = sensor_t;
        }
      }
    }
  }

  // Make sure we have enough IMU measurements to propagate to the clone time
  if (prop->imu_data.back().timestamp < tmp_clone_time || prop->imu_data.front().timestamp > tmp_clone_time) {
    PRINT0("is out of IMU measurement bound %.4f - %.4f.\n", prop->imu_data.back().timestamp, prop->imu_data.front().timestamp);
    clone_time = -1;
    return false;
  }

  // return false if we didn't find any measurements for clone
  if (isinf(min_t_diff) && !have_meas && !state->op->wheel->enabled) {
    PRINT0("cannot find measurement at or before this time.\n");
    clone_time = -1;
    return false;
  }

  // Overwrite the clone hz and order if enabled
  if (state->op->dynamic_cloning) {
    state->op->clone_freq = freq;
    state->op->intr_order = intr;
  }

  // All good. return clone time.
  clone_time = tmp_clone_time;
  assert(clone_time >= state->time);
  PRINT0("Getting clone time success.\n");
  PRINT0("%.4f %.4f %.4f %d\n", state->time, state->est_A->mean, state->est_a->mean, state->op->clone_freq);
  return true;
}

void SystemManager::compute_accelerations() {
  // Get last two IMU messages
  if (prop->imu_data.size() < 2)
    return;
  auto imu1 = prop->imu_data.end()[-2];

  if (!state->have_cpi(imu1.timestamp))
    return;

  // Compute average linear acceleration
  State::CPI cpi1 = state->cpis.at(imu1.timestamp);
  Matrix3d R_I0toG = state->clones.at(cpi1.clone_t)->Rot().transpose();
  Matrix3d R_IktoI0 = cpi1.R_I0toIk.transpose();
  Vector3d aIinG = R_I0toG * R_IktoI0 * imu1.am - state->imu->Rot().transpose() * state->imu->bias_a() - state->op->gravity;
  state->est_a->add_stat((float)aIinG.norm());

  // Compute average angular acceleration
  if (cpi1.dt != 0) {
    state->est_A->reset();
    State::CPI cpi0 = state->cpis.at(cpi1.clone_t);
    state->est_A->add_stat((float)((R_IktoI0 * cpi1.w - cpi0.w) / cpi1.dt).norm());
  }
}

void SystemManager::dynamic_cloning(int &clone_freq, int &intr_order) {
  // Based on ang., lin. accelerations, decide clone hz and interpolation order
  vector<int> clone_hzs = state->op->intr_err.available_clone_hz();
  vector<int> intr_orders = {3}; // We use 3 order for the best efficiency and accuracy

  // Loop to find best
  clone_freq = clone_hzs.back();
  intr_order = 3;
  for (auto hz : clone_hzs) {
    if (hz < 4)
      continue;
    for (auto order : intr_orders) {
      if (state->intr_ori_std(hz, order) < state->op->intr_err.threshold_ori && state->intr_pos_std(hz, order) < state->op->intr_err.threshold_pos) {
        clone_freq = hz;
        intr_order = order;
        return;
      }
    }
  }
}

void SystemManager::print_status() {
  // Debug, print our current state
  // Make sure we have at least two clones, for travel distance computation.
  double second_newest;
  if (!state->closest_older_clone_time(state->newest_clone_time(), second_newest))
    return;

  // print Chi results of the updates
  if (state->op->cam->enabled) {
    for (int cam_id = 0; cam_id < state->op->cam->max_n; cam_id++)
      up_cam->Chi.at(cam_id)->print();
  }
  if (state->op->wheel->enabled) {
    up_whl->Chi->print();
  }

  if (state->op->gps->enabled) {
    for (int gps_id = 0; gps_id < state->op->gps->max_n; gps_id++)
      up_gps->Chi.at(gps_id)->print();
  }

  // Print time debugging print
  (*state->tc)++.print_all(false, false, true);

  // Add travel distance
  distance += (state->clones.at(second_newest)->pos() - state->clones.at(state->newest_clone_time())->pos()).norm();

  // Print current state
  Vector4d q = state->imu->quat();
  Vector3d p = state->imu->pos();
  PRINT2("%.3f (%.3fs) ", state->time, state->time - state->startup_time);
  PRINT2("| q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f ", q(0), q(1), q(2), q(3), p(0), p(1), p(2));
  PRINT2("| dist = %.2f (m)\n", distance);

  // Print processing time
  avg_order->add_stat(state->op->intr_order);
  avg_freq->add_stat(state->clones.size() / state->op->window_size);
  PRINT2("Avg processing time(Curr Intr %d:%d Hz | Avg Intr %d:%d Hz): ", state->op->intr_order, state->op->clone_freq, (int)avg_order->mean, (int)avg_freq->mean);
  tc_sensors->print_all_one_line(false, true, 1 / avg_freq->mean * 1000);

  // IMU
  if (prop->t_hist.size() > 2)
    PRINT2("Hz average: IMU %.1f", (prop->t_hist.size() - 1) / (prop->t_hist.back() - prop->t_hist.front()));

  // Camera
  auto c_op = state->op->cam;
  for (int i = 0; i < c_op->max_n; i++) {
    if (c_op->enabled && up_cam->t_hist.at(i).size() > 2) {
      PRINT2(" CAM%d %.1f", i, (up_cam->t_hist.at(i).size() - 1) / (up_cam->t_hist.at(i).back() - up_cam->t_hist.at(i).front()));
    }
  }

  // GPS
  auto g_op = state->op->gps;
  for (int i = 0; i < g_op->max_n; i++) {
    if (g_op->enabled && up_gps->t_hist.at(i).size() > 2) {
      PRINT2(" GPS%d %.1f", i, (up_gps->t_hist.at(i).size() - 1) / (up_gps->t_hist.at(i).back() - up_gps->t_hist.at(i).front()));
    }
  }

  // Wheel
  auto w_op = state->op->wheel;
  if (w_op->enabled && up_whl->t_hist.size() > 2)
    PRINT2(" WHL %.1f", (up_whl->t_hist.size() - 1) / (up_whl->t_hist.back() - up_whl->t_hist.front()));

  PRINT2("\n");

  // camera
  if (c_op->enabled && (c_op->do_calib_dt || c_op->do_calib_ext || c_op->do_calib_int)) {
    for (int j = 0; j < c_op->max_n; j++) {
      PRINT2("cam%d:\n", j);
      if (c_op->do_calib_dt) {
        PRINT2("  timeoffset: %6.3f <= %6.3f\n", state->cam_dt.at(j)->value()(0), c_op->dt.at(j));
      }
      if (c_op->do_calib_ext) {
        PRINT2("  T_imu_cam:\n");
        Matrix3d r = state->cam_extrinsic.at(j)->Rot().transpose(); // R_CtoI
        Vector3d p = -r * state->cam_extrinsic.at(j)->pos();        // p_CinI
        Matrix3d R = quat_2_Rot(c_op->extrinsics.at(j).block(0, 0, 4, 1)).transpose();
        Vector3d P = -R * c_op->extrinsics.at(j).block(4, 0, 3, 1);
        PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]    [%6.3f,%6.3f,%6.3f,%6.3f]\n", r(0), r(3), r(6), p(0), R(0), R(3), R(6), P(0));
        PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f] <= [%6.3f,%6.3f,%6.3f,%6.3f]\n", r(1), r(4), r(7), p(1), R(1), R(4), R(7), P(1));
        PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]    [%6.3f,%6.3f,%6.3f,%6.3f]\n", r(2), r(5), r(8), p(2), R(2), R(5), R(8), P(2));
        PRINT2("    - [ 0.000, 0.000, 0.000, 1.000]    [ 0.000, 0.000, 0.000, 1.000]\n");
      }
      if (c_op->do_calib_int) {
        VectorXd i = state->cam_intrinsic.at(j)->value();
        VectorXd I = c_op->intrinsics.at(j);
        i.block(0, 0, 4, 1) *= c_op->downsample ? 2 : 1;
        I.block(0, 0, 4, 1) *= c_op->downsample ? 2 : 1;
        PRINT2("  intrinsics: [%.3f,%.3f,%.3f,%.3f] <= [%.3f,%.3f,%.3f,%.3f]\n", i(0), i(1), i(2), i(3), I(0), I(1), I(2), I(3));
        PRINT2("  distortion_coeffs: [%.3f,%.3f,%.3f,%.3f] <= [%.3f,%.3f,%.3f,%.3f]\n", i(4), i(5), i(6), i(7), I(4), I(5), I(6), I(7));
      }
    }
  }

  // GPS
  if (g_op->enabled && (g_op->do_calib_dt || g_op->do_calib_ext)) {
    for (int j = 0; j < g_op->max_n; j++) {
      PRINT2("gps%d:\n", j);
      if (g_op->do_calib_dt) {
        PRINT2("  timeoffset: %6.3f <= %6.3f\n", state->gps_dt.at(j)->value()(0), g_op->dt.at(j));
      }
      if (g_op->do_calib_ext) {
        Vector3d p = state->gps_extrinsic.at(j)->value(); // p_GinI
        Vector3d P = g_op->extrinsics.at(j);
        PRINT2("  pGinI: [%6.3f,%6.3f,%6.3f] <= [%6.3f,%6.3f,%6.3f]\n", p(0), p(1), p(2), P(0), P(1), P(2));
      }
    }
  }

  // wheel
  if (w_op->enabled && (w_op->do_calib_dt || w_op->do_calib_ext || w_op->do_calib_int)) {
    PRINT2("wheel:\n");
    if (w_op->do_calib_dt) {
      PRINT2("  timeoffset: %6.3f <= %6.3f\n", state->wheel_dt->value()(0), w_op->dt);
    }
    if (w_op->do_calib_ext) {
      PRINT2("  T_imu_wheel:\n");
      Matrix3d r = state->wheel_extrinsic->Rot().transpose(); // R_WtoI
      Vector3d p = -r * state->wheel_extrinsic->pos();        // p_WinI
      Matrix3d R = quat_2_Rot(w_op->extrinsics.block(0, 0, 4, 1)).transpose();
      Vector3d P = -R * w_op->extrinsics.block(4, 0, 3, 1);
      PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]    [%6.3f,%6.3f,%6.3f,%6.3f]\n", r(0), r(3), r(6), p(0), R(0), R(3), R(6), P(0));
      PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f] <= [%6.3f,%6.3f,%6.3f,%6.3f]\n", r(1), r(4), r(7), p(1), R(1), R(4), R(7), P(1));
      PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]    [%6.3f,%6.3f,%6.3f,%6.3f]\n", r(2), r(5), r(8), p(2), R(2), R(5), R(8), P(2));
      PRINT2("    - [ 0.000, 0.000, 0.000, 1.000]    [ 0.000, 0.000, 0.000, 1.000]\n");
    }
    if (w_op->do_calib_int) {
      VectorXd i = state->wheel_intrinsic->value();
      VectorXd I = w_op->intrinsics;
      PRINT2("  intrinsics: [%6.3f,%6.3f,%6.3f]  <=  [%6.3f,%6.3f,%6.3f]\n", i(0), i(1), i(2), I(0), I(1), I(2));
    }
  }
  PRINT2("\n");
}

void SystemManager::visualize_final() {

  viw::Print_Logger::close_file();
  PRINT2(BOLDYELLOW);
  // Print total travel time
  PRINT2("\n========Final Status========\n");
  PRINT2("Total procesing time: %ds\n", (int)tc_sensors->get_total_sum());
  PRINT2("Total traveling time: %ds\n", (int)(state->time - state->startup_time));

  // print calibration
  // camera
  auto c_op = state->op->cam;
  if (c_op->enabled && (c_op->do_calib_dt || c_op->do_calib_ext || c_op->do_calib_int)) {
    for (int j = 0; j < c_op->max_n; j++) {
      PRINT2("cam%d:\n", j);
      if (c_op->do_calib_dt) {
        PRINT2("  timeoffset: %6.3f\n", state->cam_dt.at(j)->value()(0));
      }
      if (c_op->do_calib_ext) {
        PRINT2("  T_imu_cam:\n");
        Matrix3d R = state->cam_extrinsic.at(j)->Rot().transpose(); // R_CtoI
        Vector3d p = -R * state->cam_extrinsic.at(j)->pos();        // p_CinI
        PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]\n", R(0), R(3), R(6), p(0));
        PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]\n", R(1), R(4), R(7), p(1));
        PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]\n", R(2), R(5), R(8), p(2));
        PRINT2("    - [ 0.000, 0.000, 0.000, 1.000]\n");
      }
      if (c_op->do_calib_int) {
        VectorXd i = state->cam_intrinsic.at(j)->value();
        i.block(0, 0, 4, 1) *= state->op->cam->downsample ? 2 : 1;
        PRINT2("  intrinsics: [%.5f,%.5f,%.5f,%.5f]\n", i(0), i(1), i(2), i(3));
        PRINT2("  distortion_coeffs: [%.5f,%.5f,%.5f,%.5f]\n", i(4), i(5), i(6), i(7));
      }
    }
  }
  // GPS
  auto g_op = state->op->gps;
  if (g_op->enabled && (g_op->do_calib_dt || g_op->do_calib_ext)) {
    for (int j = 0; j < g_op->max_n; j++) {
      PRINT2("gps%d:\n", j);
      if (g_op->do_calib_dt) {
        PRINT2("  timeoffset: %6.3f\n", state->gps_dt.at(j)->value()(0));
      }
      if (g_op->do_calib_ext) {
        Vector3d p = state->gps_extrinsic.at(j)->value(); // p_GinI
        PRINT2("  pGinI: [%6.3f,%6.3f,%6.3f]\n", p(0), p(1), p(2));
      }
    }
  }
  // wheel
  auto w_op = state->op->wheel;
  if (w_op->enabled && (w_op->do_calib_dt || w_op->do_calib_ext || w_op->do_calib_int)) {
    PRINT2("wheel:\n");
    if (w_op->do_calib_dt) {
      PRINT2("  timeoffset: %6.3f\n", state->wheel_dt->value()(0));
    }
    if (w_op->do_calib_ext) {
      PRINT2("  T_imu_wheel:\n");
      Matrix3d R = state->wheel_extrinsic->Rot().transpose(); // R_WtoI
      Vector3d p = -R * state->wheel_extrinsic->pos();        // p_WinI
      PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]\n", R(0), R(3), R(6), p(0));
      PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]\n", R(1), R(4), R(7), p(1));
      PRINT2("    - [%6.3f,%6.3f,%6.3f,%6.3f]\n", R(2), R(5), R(8), p(2));
      PRINT2("    - [ 0.000, 0.000, 0.000, 1.000]\n");
    }
    if (w_op->do_calib_int) {
      Vector3d i = state->wheel_intrinsic->value();
      PRINT2("  intrinsics: [%.5f,%.5f,%.5f]\n", i(0), i(1), i(2));
    }
  }
  PRINT2("\n" RESET);
}
