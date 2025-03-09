#include "UpdaterCamera.h"
#include "CamHelper.h"
#include "CamTypes.h"
#include "linefeat/LineType.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "TrackLSD.h"
#include "linefeat/LineHelper.h"
#include "types/Landmark.h"
#include "update/UpdaterStatistics.h"
#include "utils/Print_Logger.h"
#include "utils/TimeChecker.h"
#include "linefeat/LineFeature.h"

using namespace std;
using namespace Eigen;
using namespace ov_core;
using namespace viw;

UpdaterCamera::UpdaterCamera(shared_ptr<State> state) : state(state) {
  shared_ptr<OptionsCamera> op = state->op->cam;
  // Let's make a feature extractor and other setups
  for (int i = 0; i < op->max_n; i++) {
    // check if we have the trackDATABASE for this camera
    if (trackDATABASE.find(i) == trackDATABASE.end()) {
      // not found
      // check if this is stereo camera
      if (!op->use_stereo || op->stereo_pairs.find(i) == op->stereo_pairs.end()) {
        // this is mono
        // Set up the system
        t_hist.insert({i, deque<double>()});
        Chi.insert({i, make_shared<UpdaterStatistics>(op->chi2_mult, "CAM", i)});
        trackDATABASE.insert({i, make_shared<FeatureDatabase>()});
        trackFEATS.insert(
            {i, shared_ptr<TrackBase>(new TrackKLT(state->cam_intrinsic_model, op->n_pts, 0, op->use_stereo, op->histogram, op->fast, op->grid_x, op->grid_y, op->min_px_dist))});
        if (op->use_lines) {
          point_used = make_shared<FeatureDatabase>();
          trackLSDS.insert({i, shared_ptr<TrackLSD>(new TrackLSD(state->cam_intrinsic_model, op->use_stereo, op->histogram, trackFEATS))});
        }
      } else {
        // this is stereo
        // check if we have the system setup for the paired camera
        if (trackDATABASE.find(op->stereo_pairs.at(i)) != trackDATABASE.end()) {
          // we have
          // copy over
          t_hist.insert({i, t_hist.at(op->stereo_pairs.at(i))});
          Chi.insert({i, Chi.at(op->stereo_pairs.at(i))});
          trackDATABASE.insert({i, trackDATABASE.at(op->stereo_pairs.at(i))});
          trackFEATS.insert({i, trackFEATS.at(op->stereo_pairs.at(i))});
        } else {
          // we do not have
          // Set up the system
          t_hist.insert({i, deque<double>()});
          Chi.insert({i, make_shared<UpdaterStatistics>(op->chi2_mult, "CAM", i)});
          trackDATABASE.insert({i, make_shared<FeatureDatabase>()});
          trackFEATS.insert(
              {i, shared_ptr<TrackBase>(new TrackKLT(state->cam_intrinsic_model, op->n_pts, 0, op->use_stereo, op->histogram, op->fast, op->grid_x, op->grid_y, op->min_px_dist))});
        }
      }
    } else {
      // found.
      // Make sure it is the same trackDATABASE of the other stereo paired camera
      assert(trackDATABASE.at(i) == trackDATABASE.at(op->stereo_pairs.at(i)));
    }
  }

  // init timing logger
  tc = std::make_shared<TimeChecker>();
}

void UpdaterCamera::feed_measurement(const ov_core::CameraData &camdata) {
  // Assert we have valid measurement data and ids
  tc->dingdong("[Time-Cam] feed measurement");
  assert(!camdata.sensor_ids.empty());
  assert(camdata.sensor_ids.size() == camdata.images.size());
  for (size_t i = 0; i < camdata.sensor_ids.size() - 1; i++) {
    assert(camdata.sensor_ids.at(i) != camdata.sensor_ids.at(i + 1));
  }
  // Downsample if we are downsampling
  ov_core::CameraData message = camdata;
  for (size_t i = 0; i < message.sensor_ids.size() && state->op->cam->downsample; i++) {
    cv::Mat img = message.images.at(i);
    cv::Mat mask = message.masks.at(i);
    cv::Mat img_temp, mask_temp;
    cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
    message.images.at(i) = img_temp;
    cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
    message.masks.at(i) = mask_temp;
  }

  // record timestamps
  for (auto cam_id : camdata.sensor_ids) {
    t_hist.at(cam_id).size() > 100 ? t_hist.at(cam_id).pop_front() : void(); // remove if we have too many
    t_hist.at(cam_id).push_back(camdata.timestamp);
  }

  // Perform our feature tracking!
  int cam_id = message.sensor_ids.at(0);
  trackFEATS.at(cam_id)->feed_new_camera(message);
  if (!(trackLSDS.find(cam_id) == trackLSDS.end())) {
    std::vector<Eigen::Vector2d> vanishing_points(3);
    LineHelper::Vanishing_Points(state, vanishing_points, message);
    trackLSDS.at(cam_id)->feed_new_camera(message, vanishing_points);
  }
  trackDATABASE.at(cam_id)->append_new_measurements(trackFEATS.at(cam_id)->get_feature_database());
  
  // Marginalize lost-track SLAM features
  marginalize_slam_features(camdata);
  tc->dingdong("[Time-Cam] feed measurement");
}

void UpdaterCamera::marginalize_slam_features(const ov_core::CameraData &camdata) {
  // Loop through current SLAM features, we have tracks of them, grab them for this update!
  // Note: if we have a slam feature that has lost tracking, then we should marginalize it out
  // Note: we only enforce this if the current camera message is where the feature was seen from
  // Note: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
  for (pair<const size_t, shared_ptr<ov_type::Landmark>> &slam_feat : state->cam_SLAM_features) {
    auto landmark = slam_feat.second;
    shared_ptr<Feature> feat = trackFEATS.at(camdata.sensor_ids.at(0))->get_feature_database()->get_feature(landmark->_featid);
    assert(landmark->_unique_camera_id != -1);
    bool current_unique_cam = find(camdata.sensor_ids.begin(), camdata.sensor_ids.end(), landmark->_unique_camera_id) != camdata.sensor_ids.end();
    if (feat == nullptr && current_unique_cam)
      landmark->should_marg = true;
    if (landmark->update_fail_count > 1)
      landmark->should_marg = true;
  }

  // Let's marginalize out all old SLAM features here
  // These are ones that where not successfully tracked into the current frame
  StateHelper::marginalize_slam(state);
}

void UpdaterCamera::try_update(int cam_id) {
  // Return if we do not have polynomial constructed yet.
  if (!state->have_polynomial())
    return;

  // Get visual features to update!
  V_Feature msckf, slam, init;
  // DB_ptr point_used = make_shared<FeatureDatabase>();
  tc->dingdong("[Time-Cam] get features");
  CamHelper::get_features(state, msckf, slam, init, trackFEATS.at(cam_id)->get_feature_database(), t_hist.at(cam_id), point_used);

  // Get visual line feature to update
  std::vector<shared_ptr<LineFeature>> lines;
  LineHelper::get_line_features(state, lines, trackLSDS.at(cam_id)->get_feature_database(), t_hist.at(cam_id), point_used);
  
  tc->dingdong("[Time-Cam] get features");

  // collect unused features during the update process
  DB_ptr db_unused = make_shared<FeatureDatabase>();

  // MSCKF update
  tc->dingdong("[Time-Cam] MSCKF update");
  msckf_update(msckf, db_unused);
  tc->dingdong("[Time-Cam] MSCKF update");

  // SLAM update
  tc->dingdong("[Time-Cam] SLAM update");
  slam_update(slam, db_unused);
  tc->dingdong("[Time-Cam] SLAM update");

  // SLAM feature initialization
  tc->dingdong("[Time-Cam] SLAM init");
  slam_init(init, db_unused);
  tc->dingdong("[Time-Cam] SLAM init");

  // cleanup used features.
  tc->dingdong("[Time-Cam] DB clan up");
  CamHelper::cleanup_features(state, msckf, db_unused, msckf_used, trackFEATS.at(cam_id), trackDATABASE.at(cam_id));

  // Line update
  shared_ptr <LineFeatureDatabase> ldb_unused = make_shared<LineFeatureDatabase>();
  tc->dingdong("[Time-Cam] LINE update");
  lines_update(lines, ldb_unused);
  tc->dingdong("[Time-Cam] LINE update");

  // Cleanup used lines 
  LineHelper::cleanup_lines(state, lines, ldb_unused, lines_used, trackLSDS.at(cam_id));
  // Cleanup any features older than the marginalization time
  if (state->clone_window() > state->op->window_size) {
    point_used->cleanup_measurements(state->oldest_clone_time());
  }
  tc->dingdong("[Time-Cam] DB clan up");
  tc->counter++;

  if (state->op->cam->time_analysis)
    tc->print_all();
}

void UpdaterCamera::msckf_update(V_Feature &vec_features, DB_ptr db_unused) {
  // Calculate the max possible measurement size
  int max_meas_size = 0;
  for (auto &feat : vec_features) {
    for (const auto &pair : feat->timestamps) {
      max_meas_size += 2 * feat->timestamps[pair.first].size();
    }
  }

  // Calculate max possible state size (i.e. the size of our covariance)
  // NOTE: that when we have the single inverse depth representations, those are only 1dof in size
  int max_hx_size = state->cov.rows();
  for (auto &landmark : state->cam_SLAM_features)
    max_hx_size -= landmark.second->size();

  // Large Jacobian and residual of *all* features for this update
  CamLinSys LL; // linear system large
  LL.res = VectorXd::Zero(max_meas_size);
  LL.Hx = MatrixXd::Zero(max_meas_size, max_hx_size);
  int ct_jacob = 0;
  int ct_meas = 0;
  ID_T_POSE imu_poses;
  CamHelper::get_imu_poses(state, vec_features, db_unused, imu_poses);

  for (auto feat = vec_features.begin(); feat != vec_features.end();) {

    // Convert our feature into our current format
    CamFeature cam_feat = CamHelper::create_feature((*feat), state->op->cam->feat_rep);
    // get linear system for this feature
    CamLinSys L = CamHelper::get_feature_jacobian_full(state, cam_feat, db_unused, imu_poses);
    // make sure if we have linear system with more than 1 measurement
    if (L.res.size() < 4) {
      feat++;
      continue;
    }

    // Nullspace projection
    StateHelper::nullspace_project_inplace(L.Hf, L.Hx, L.res);

    // Get noise covariance R
    L.R = MatrixXd::Zero(L.res.rows(), L.res.rows());
    L.R.diagonal() += pow(state->op->cam->sigma_pix, 2) * Eigen::VectorXd::Ones(L.R.rows());

    // Check if we should delete or not
    int cam_id = (int)(*feat)->timestamps.begin()->first;
    if (L.res.norm() < 3 && Chi.at(cam_id)->Chi2Check(state, L.Hx_order, L.Hx, L.res, L.R)) {
      // set the Chi_test to true for display
      if (trackFEATS.at(cam_id)->get_feature_database()->get_internal_data().find((*feat)->featid) != trackFEATS.at(cam_id)->get_feature_database()->get_internal_data().end()) {
        trackFEATS.at(cam_id)->get_feature_database()->get_internal_data().at((*feat)->featid)->Chi_test = true;
      }
      size_t ct_hx = 0;
      for (const auto &var : L.Hx_order) {
        // Ensure that this variable is in our Jacobian
        if (LL.Hx_mapping.find(var) == LL.Hx_mapping.end()) {
          LL.Hx_mapping.insert({var, ct_jacob});
          LL.Hx_order.push_back(var);
          ct_jacob += var->size();
        }

        // Append to our large Jacobian
        LL.Hx.block(ct_meas, LL.Hx_mapping[var], L.Hx.rows(), var->size()) = L.Hx.block(0, ct_hx, L.Hx.rows(), var->size());
        ct_hx += var->size();
      }

      // Append our residual and Covarinace, and move forward
      LL.res.block(ct_meas, 0, L.res.rows(), 1) = L.res;
      ct_meas += L.res.rows();
    } else {
      // case fail
      if (trackFEATS.at(cam_id)->get_feature_database()->get_internal_data().find((*feat)->featid) != trackFEATS.at(cam_id)->get_feature_database()->get_internal_data().end()) {
        trackFEATS.at(cam_id)->get_feature_database()->get_internal_data().at((*feat)->featid)->Chi_test = false;
      }
      CamHelper::copy_to_db(db_unused, (*feat));
    }
    feat++;
  }

  // Return if we don't have anything and resize our matrices
  if (ct_meas < 1) {
    return;
  }

  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);
  LL.res.conservativeResize(ct_meas, 1);
  LL.Hx.conservativeResize(ct_meas, ct_jacob);

  // 5. Perform measurement compression
  StateHelper::measurement_compress_inplace(LL.Hx, LL.res);
  if (LL.Hx.rows() < 1)
    return;

  // Our noise is whitened, so make it here after our compression
  LL.R = MatrixXd::Identity(LL.res.rows(), LL.res.rows());

  // 6. With all good features update the state
  StateHelper::EKFUpdate(state, LL.Hx_order, LL.Hx, LL.res, LL.R, "CAM");
}

void UpdaterCamera::slam_update(V_Feature &vec_features, DB_ptr db_unused) {
  for (auto feat = vec_features.begin(); feat != vec_features.end();) {
    // Ensure we have the landmark and it is the same
    assert(state->cam_SLAM_features.find((*feat)->featid) != state->cam_SLAM_features.end());
    assert(state->cam_SLAM_features.at((*feat)->featid)->_featid == (*feat)->featid);

    // Get the IMU poses
    ID_T_POSE imu_poses;
    CamHelper::get_imu_poses(state, (*feat), db_unused, imu_poses);

    // Convert the state landmark into our current format
    shared_ptr<ov_type::Landmark> landmark = state->cam_SLAM_features.at((*feat)->featid);
    CamFeature cam_feat = CamHelper::create_feature((*feat), landmark);

    // Get the Jacobian for this feature
    CamLinSys L = CamHelper::get_feature_jacobian_full(state, cam_feat, db_unused, imu_poses);

    // make sure if we have linear system with >0 measurements
    if (L.res.size() < 2) {
      feat++;
      continue;
    }

    // Append to our Jacobian order vector
    L.Hx_order.push_back(landmark);

    // Place Jacobians in one big Jacobian, since the landmark is already in our state vector
    MatrixXd H_xf = L.Hx;
    H_xf.conservativeResize(L.Hx.rows(), L.Hx.cols() + L.Hf.cols());
    H_xf.block(0, L.Hx.cols(), L.Hx.rows(), L.Hf.cols()) = L.Hf;

    // Check if we should delete or not
    int cam_id = (int)(*feat)->timestamps.begin()->first;
    if (Chi.at(cam_id)->Chi2Check(state, L.Hx_order, H_xf, L.res, L.R)) {
      // 5. With all good SLAM features update the state
      StateHelper::EKFUpdate(state, L.Hx_order, H_xf, L.res, L.R, "CAM");
    } else {
      // case fail return to database
      //      CamHelper::copy_to_db(db_unused, (*feat));
    }
    feat++;
  }
}

void UpdaterCamera::slam_init(V_Feature &vec_features, DB_ptr db_unused) {
  // Compute linear system for each feature, nullspace project, and reject
  for (auto feat = vec_features.begin(); feat != vec_features.end();) {
    // Get the IMU poses
    ID_T_POSE imu_poses;
    CamHelper::get_imu_poses(state, (*feat), db_unused, imu_poses);

    // Convert our feature into our current format
    CamFeature cam_feat = CamHelper::create_feature((*feat), state->op->cam->feat_rep);

    // Get the Jacobian for this feature
    CamLinSys L = CamHelper::get_feature_jacobian_full(state, cam_feat, db_unused, imu_poses);

    // make sure if we have linear system with >1 measurements
    if (L.res.size() < 4) {
      feat++;
      continue;
    }

    // Try to initialize, delete new pointer if we failed
    shared_ptr<ov_type::Landmark> landmark = CamHelper::create_landmark(cam_feat);
    if (StateHelper::initialize(state, landmark, L.Hx_order, L.Hx, L.Hf, L.R, L.res, state->op->cam->chi2_mult, "CAM")) {
      state->cam_SLAM_features.insert({(*feat)->featid, landmark});
    } else {
      // case fail
      CamHelper::copy_to_db(db_unused, (*feat));
    }
    feat++;
  }
}

void UpdaterCamera::lines_update (vector<shared_ptr<LineFeature>> &line_features, shared_ptr <LineFeatureDatabase> lbd_unused) {
  // Calculate the max possible measurement size
  bool PLC = false;
  int max_meas_size = 0;
  for (auto &line_feat : line_features) {
    for (const auto &pair : line_feat->timestamps) {
      max_meas_size += 4 * line_feat->timestamps[pair.first].size();
      if (PLC) {
        for (const auto &feats : line_feat->point_uvs[pair.first]) {
          max_meas_size += 2 * feats.second.size();
        }
      }
    }
  }

  // Calculate max possible state size (i.e. the size of our covariance)
  // NOTE: that when we have the single inverse depth representations, those are only 1dof in size
  int max_hx_size = state->cov.rows();
  for (auto &landmark : state->cam_SLAM_features)
    max_hx_size -= landmark.second->size();

  // Large Jacobian and residual of *all* features for this update
  LineLinSys LL; // line linear system large
  LL.res = VectorXd::Zero(max_meas_size);
  LL.Hx = MatrixXd::Zero(max_meas_size, max_hx_size);
  int ct_jacob = 0;
  int ct_meas = 0;

  ID_T_POSE imu_poses;
  LineHelper::get_imu_poses(state, line_features, lbd_unused, imu_poses);

  for (auto line_feat = line_features.begin(); line_feat != line_features.end();) {
    // get linear system for this feature
    LineLinSys L = LineHelper::get_line_feature_jacobian_full(state, (*line_feat), lbd_unused, imu_poses, PLC);
    // make sure if we have linear system with more than 1 measurement
    if (L.res.size() < 5) {
      line_feat++;
      continue;
    }

    // Nullspace projection
    StateHelper::nullspace_project_inplace(L.Hf, L.Hx, L.res);
    
    // Get noise covariance R
    L.R = MatrixXd::Zero(L.res.rows(), L.res.rows());
    L.R.diagonal() += pow(state->op->cam->sigma_pix, 2) * Eigen::VectorXd::Ones(L.R.rows());
    
    // Check if we should delete or not
    int cam_id = (int)(*line_feat)->timestamps.begin()->first;
    if (Chi.at(cam_id)->Chi2Check(state, L.Hx_order, L.Hx, L.res, L.R)) {
    // if (L.res.norm() < 3 && Chi.at(cam_id)->Chi2Check(state, L.Hx_order, L.Hx, L.res, L.R)) {
      size_t ct_hx = 0;
      for (const auto &var : L.Hx_order) {
        // Ensure that this variable is in our Jacobian
        if (LL.Hx_mapping.find(var) == LL.Hx_mapping.end()) {
          LL.Hx_mapping.insert({var, ct_jacob});
          LL.Hx_order.push_back(var);
          ct_jacob += var->size();
        }
        // Append to our large Jacobian
        LL.Hx.block(ct_meas, LL.Hx_mapping[var], L.Hx.rows(), var->size()) = L.Hx.block(0, ct_hx, L.Hx.rows(), var->size());
        ct_hx += var->size();
      }      
      
      // Append our residual and Covarinace, and move forward
      LL.res.block(ct_meas, 0, L.res.rows(), 1) = L.res;
      ct_meas += L.res.rows();      
    } else {
      // case fail
      LineHelper::copy_to_db(lbd_unused, (*line_feat));      
    }
    line_feat++;
  }
  // Return if we don't have anything and resize our matrices
  if (ct_meas < 1) {
    return;
  }

  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);
  LL.res.conservativeResize(ct_meas, 1);
  LL.Hx.conservativeResize(ct_meas, ct_jacob);

  // 5. Perform measurement compression
  StateHelper::measurement_compress_inplace(LL.Hx, LL.res);
  if (LL.Hx.rows() < 1)
    return;

  // Our noise is whitened, so make it here after our compression
  LL.R = MatrixXd::Identity(LL.res.rows(), LL.res.rows());

  // 6. With all good features update the state
  StateHelper::EKFUpdate(state, LL.Hx_order, LL.Hx, LL.res, LL.R, "LINE");
}

vector<Eigen::Vector3d> UpdaterCamera::get_used_msckf() {
  // copy used msckf
  vector<Vector3d> msckf_used_cp = msckf_used;

  // clear history
  msckf_used.clear();

  // return
  return msckf_used_cp;
}

vector<Eigen::Matrix<double, 6, 1>> UpdaterCamera::get_used_lines() {
  // copy used lines
  vector<Eigen::Matrix<double, 6, 1>> lines_used_cp;
  lines_used_cp = lines_used;
  // clear history
  // lines_used.clear();
  return lines_used_cp;
}

cv::Mat UpdaterCamera::get_track_img(int cam_id) {

  // Build an id-list of what features we should highlight (i.e. SLAM)
  std::vector<size_t> highlighted_ids;
  for (const auto &feat : state->cam_SLAM_features) {
    highlighted_ids.push_back(feat.first);
  }

  // Text we will overlay if needed
  std::string overlay = (!state->initialized) ? "init" : "";

  cv::Mat img_history;
  trackFEATS.at(cam_id)->display_history(img_history, 255, 255, 0, 255, 255, 255, highlighted_ids, overlay);
  return img_history;
}