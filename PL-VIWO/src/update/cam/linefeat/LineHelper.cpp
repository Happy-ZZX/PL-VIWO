#include "LineHelper.h"
#include "state/State.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "update/cam/TrackLSD.h"
#include "options/OptionsEstimator.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "LineType.h"
#include "utils/Jabdongsani.h"
#include "options/OptionsCamera.h"
#include "types/IMU.h"

using namespace std;
using namespace Eigen;
using namespace viw;
using namespace ov_core;

void LineHelper::get_line_features(State_ptr state, L_Feature &line, LDB_ptr ldb_feats, deque<double> t_hist, DB_ptr point_database) {
  // return if we have just one image so far
  if (t_hist.size() < 2)
    return;

  // Init un-usable line feature db
  LDB_ptr db_unused = make_shared<LineFeatureDatabase>();

  ldb_feats->cleanup();                                  // Remove extracted features from the database so there would be no duplicate.

  // The feature that contains in the sliding window
  L_Feature feats_pool;

  // Get features that have measurements older than second-oldest clone
  features_containing_older(state, ldb_feats, state->oldest_2nd_clone_time(), feats_pool);
  ldb_feats->cleanup();

  // Get features that have no measurement at the newest frame
  features_not_containing_newer(state, ldb_feats, t_hist.at(t_hist.size() - 2), feats_pool);
  ldb_feats->cleanup();
  remove_unusable_measurements(state, feats_pool, db_unused); // return un-usable measurements to db

  // Sort based on track length long to short 
  // ( which make it easy to use feature with more observation to update)
  sort(feats_pool.begin(), feats_pool.end(), feat_sort);

  // Loop through features and decide what to do with them
  ID_T_POSE imu_poses, cam_poses;
  for (auto feat = feats_pool.begin(); feat != feats_pool.end();) {

    // move on if not enough measurement
    if (n_meas(*feat) < 2) {
      copy_to_db(db_unused, (*feat));
      feat = feats_pool.erase(feat);
      continue;
    }

    // Triangulation
    get_imu_poses(state, (*feat), db_unused, imu_poses);
    get_cam_poses(state, imu_poses, cam_poses);

    if (!line_triangulation(*feat, imu_poses, cam_poses, false, point_database)) {
      copy_to_db(db_unused, (*feat));
      feat = feats_pool.erase(feat);
      continue;
    }

    line.push_back(*feat);
    feat = feats_pool.erase(feat);
  }
  // Move remaining features back to db
  copy_to_db(db_unused, feats_pool);
  ldb_feats->append_new_measurements(db_unused);
}

void LineHelper::features_not_containing_newer(shared_ptr<State> state, LDB_ptr ldb, double timestamp, L_Feature &feat_found) {
  // Now lets loop through all features, and just make sure they are not old
  auto feats = ldb->get_internal_data();
  for (auto it = feats.begin(); it != feats.end();) {
    // Skip if already deleted
    if ((*it).second->to_delete) {
      it++;
      continue;
    }
    // Find features that does not have measurements newer than the specified
    bool has_newer_measurement = false;
    for (auto const &pair : (*it).second->timestamps) {
      for (auto meas_t : pair.second) {
        if (meas_t > timestamp - state->cam_dt.at(pair.first)->value()(0)) {
          has_newer_measurement = true;
          break;
        }
      }
    }
    // If it is not being actively tracked, then it is old
    if (!has_newer_measurement) {
      (*it).second->to_delete = true; // mark for deletion
      feat_found.push_back((*it).second);
    }
    // move forward
    it++;
  }
}

void LineHelper::features_containing_older(State_ptr state, LDB_ptr ldb, double timestamp, L_Feature &feat_found) {
  // Now lets loop through all features, and just make sure they are not old
  auto feats = ldb->get_internal_data();
  for (auto feat = feats.begin(); feat != feats.end();) {
    // Skip if already deleted
    if ((*feat).second->to_delete) {
      feat++;
      continue;
    }
    // Find features that has measurements older than the specified
    bool found_containing_older = false;
    for (auto const &pair : (*feat).second->timestamps) {
      for (auto meas_t : pair.second) {
        if (meas_t < timestamp - state->cam_dt.at(pair.first)->value()(0)) {
          found_containing_older = true;
          break;
        }
      }
    }
    // If it has an older timestamp, then add it
    if (found_containing_older) {
      (*feat).second->to_delete = true; // mark for deletion
      feat_found.push_back((*feat).second);
    }
    // move forward
    feat++;
  }
}

void LineHelper::get_imu_poses(shared_ptr<State> state, L_Feature &l_feats, LDB_ptr ldb, ID_T_POSE &imu_poses) {
  // loop through feature measurements and compute IMU poses
  for (auto feat = l_feats.begin(); feat != l_feats.end();) {
    get_imu_poses(state, (*feat), ldb, imu_poses);
    // remove if feature has no measurements
    if (n_meas(*feat) == 0)
      feat = l_feats.erase(feat);
    // also remove feature if it has 1 measurement
    else if (n_meas(*feat) == 1)
      l_feats.erase(feat);
    else // Otherwise this is good
      feat++;
  }
}

void LineHelper::get_imu_poses(shared_ptr<State> state, shared_ptr<LineFeature> feat, LDB_ptr ldb, ID_T_POSE &imu_poses) {
  // loop through feature measurements and compute IMU poses
  vector<double> invalid_times;
  for (auto &meas : feat->timestamps) {
    int cam_id = meas.first;
    double dt = state->cam_dt.at(meas.first)->value()(0);

    // Append a new cam id if not exist
    if (imu_poses.find(cam_id) == imu_poses.end())
      imu_poses.insert({cam_id, unordered_map<double, ov_core::FeatureInitializer::ClonePose>()});

    // now loop through each measurement and compute the IMU pose
    for (auto mtime : meas.second) {
      // Check if we already have this pose
      if (imu_poses.at(cam_id).find(mtime + dt) == imu_poses.at(cam_id).end()) {
        // compute imu pose
        Matrix3d R_GtoI;
        Vector3d p_IinG;
        if (state->get_interpolated_pose(mtime + dt, R_GtoI, p_IinG)) {
          imu_poses.at(cam_id).insert({mtime, ov_core::FeatureInitializer::ClonePose(R_GtoI, p_IinG)});
        } else {
          copy_to_db(ldb, feat, meas.first, mtime);
          invalid_times.push_back(mtime);
        }
      }
    }
  }

  // remove all the measurements that do not have bounding poses
  feat->clean_invalid_measurements(invalid_times);
}

void LineHelper::get_cam_poses(shared_ptr<State> state, ID_T_POSE &imu_poses, ID_T_POSE &cam_poses){
  // Get camera pose based on n-order interpolation
  for (const auto &id_t_pose : imu_poses) {
    // Load Cam info
    int cam_id = id_t_pose.first;
    Matrix3d R_ItoC = state->cam_extrinsic.at(cam_id)->Rot();
    Vector3d p_IinC = state->cam_extrinsic.at(cam_id)->pos();

    // Append a new cam id if not exist
    if (cam_poses.find(cam_id) == cam_poses.end())
      cam_poses.insert({cam_id, unordered_map<double, ov_core::FeatureInitializer::ClonePose>()});

    // Append camera pose
    for (auto t_pose : id_t_pose.second) {
      if (cam_poses.at(cam_id).find(t_pose.first) == cam_poses.at(cam_id).end()) {
        Matrix3d R_GtoC = R_ItoC * t_pose.second.Rot();
        Vector3d p_CinG = t_pose.second.pos() - R_GtoC.transpose() * p_IinC;
        cam_poses.at(cam_id).insert({t_pose.first, ov_core::FeatureInitializer::ClonePose(R_GtoC, p_CinG)});
      }
    }
  }
}

bool LineHelper::line_triangulation(std::shared_ptr<LineFeature> &line_feat, ID_T_POSE imu_poses, ID_T_POSE cam_poses, bool optimization_refine, DB_ptr &point_database) {
  
  // return false if not enough measurement
  if (n_meas(line_feat) < 2){
    return false;
  }
  
  if (line_feat->D > 0) {
    if (line_triangulation_from_points_and_direction(line_feat, point_database, imu_poses)) {
      line_feat->triangulated = true;
      return true;
    }
  }
  
  // Attempt single-view triangulation
  if (!line_single_triangulation(line_feat, cam_poses)) {
    return false;
  }

  if (optimization_refine && !line_gaussnewton(line_feat, cam_poses)) {
    line_feat->triangulated = false;
  } else {
    line_feat->triangulated = true;
    return true;
  }
  
  return false;
}

bool LineHelper::line_triangulation_from_points_and_direction(std::shared_ptr<LineFeature> &line_feat, DB_ptr &database, ID_T_POSE imu_poses) {
  // Get the related feature points
  std::vector<int> IDs = line_feat->points;
  std::vector<Eigen::Vector3d> points;
  for (const auto& id : IDs) {
    std::shared_ptr<Feature> p_feat = database->get_feature(id);
    if (p_feat != nullptr && p_feat->Triangulated) {
      points.push_back(p_feat->p_FinG);
    } else {
      continue;
    }
  }
  if (points.size() < 1) {
    return false;
  }
  const Eigen::Matrix<double, 3, 3> &R_GtoI = imu_poses.at(0).at(line_feat->timestamps.at(0).at(0)).Rot();
  // Get the direction 
  int D = line_feat->D;
  Vector3d direction;
  if (D == 1) {
    direction = R_GtoI.transpose() * Eigen::Vector3d(1, 0, 0);
  } else if (D == 2) {
    direction = R_GtoI.transpose() * Eigen::Vector3d(0, 1, 0);
  } else if (D == 3) {
    direction = R_GtoI.transpose() * Eigen::Vector3d(0, 0, 1);
  }
  // direction.normalize();

  // Initial guess for the moment vector
  Vector3d initial_moment = points[0].cross(direction);
  double n[3] = {initial_moment(0), initial_moment(1), initial_moment(2)};

  // // Optimization Problems
  // ceres::Problem problem;
  // for (size_t i = 0; i < points.size(); ++i) {
  //   problem.AddResidualBlock(
  //     new ceres::AutoDiffCostFunction<OptimizeMomentResidual, 3, 3>(
  //         new OptimizeMomentResidual(points[i], direction)),
  //         nullptr, n);
  // }

  // // Solve the problem
  // ceres::Solver::Options options;
  // options.linear_solver_type = ceres::DENSE_QR;
  // options.minimizer_progress_to_stdout = false;
  // ceres::Solver::Summary summary;
  // ceres::Solve(options, &problem, &summary);

  // Set the Plucker coordinates
  Vector6d line_in_world;
  Vector3d norm(n[0], n[1], n[2]);
  // norm.normalize();
  line_in_world.head(3) = norm;
  line_in_world.tail(3) = direction;
  line_feat->line_FinG = line_in_world;

  // Set the end-points
  Vector3d P1 = points[0];
  Vector3d P2 = P1 + 20 * direction;
  line_feat->EndPoints.head(3) = P1; 
  line_feat->EndPoints.tail(3) = P2;
  return true;
}

bool LineHelper::line_triangulation_from_points(std::shared_ptr<LineFeature> &line_feat, DB_ptr &database){
  std::vector<int> IDs = line_feat->points;
  std::vector<Eigen::Vector3d> points; 
  Eigen::Vector3d centroid(0, 0, 0);
  int valid_n = 0;
  // calculate the centre point
  for (const auto& id : IDs) {
    shared_ptr<Feature> p_feat = database->get_feature(id);
    if (p_feat != nullptr && p_feat->Triangulated ) {
      centroid += p_feat->p_FinG;
      points.push_back(p_feat->p_FinG);
      valid_n ++;
    } else {
      continue;
    }
  }

  if (valid_n < 2){
    return false;
  }
  centroid /= valid_n;

  // Calculate the covariance matrix
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& p : points) {
      Eigen::Vector3d centered = p - centroid;
      covariance += centered * centered.transpose();
  }
  
  // Slove this Least Squares Problem
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance);
  Eigen::Vector3d direction = eigenSolver.eigenvectors().col(2);
  Eigen::Vector3d moment = centroid.cross(direction);

  // Set the Plucker coordinates
  Vector6d line_in_world;
  line_in_world.head(3) = moment;
  line_in_world.tail(3) = direction;
  line_feat->line_FinG = line_in_world;

  // Set the endpoints
  // If this line only have two points valid
  if (valid_n == 2){
    line_feat->EndPoints.head(3) = points[0];
    line_feat->EndPoints.tail(3) = points[1];
    return true;
  }
  
  // Else sort the point by distance 
  size_t md = 0;
  float max_v = std::abs(direction[0]);
  for(size_t i = 1; i < 3; i++){
    float v = std::abs(direction[i]);
    if(v > max_v){
      md = i;
      max_v = v;
    }
  }

  std::vector<size_t> order;
  order.resize(points.size());
  std::iota(order.begin(), order.end(), 0);       
  std::sort(order.begin(), order.end(), [&points, &md](size_t i1, size_t i2) {
    if(md == 0) return points[i1][0] < points[i2][0];
    if(md == 1) return points[i1][1] < points[i2][1];
    if(md == 2) return points[i1][2] < points[i2][2];
  });
  size_t min_idx = order[0], max_idx = order[(order.size()-1)];
  if ((points[max_idx] - points[min_idx]).norm() <= 20) {
    line_feat->EndPoints.head(3) = points[min_idx]; 
    line_feat->EndPoints.tail(3) = points[max_idx];
  }
  line_feat->EndPoints.head(3) = points[min_idx]; 
  line_feat->EndPoints.tail(3) = points[max_idx];
  return true;
}

bool LineHelper::line_single_triangulation(std::shared_ptr<LineFeature> &line_feat, ID_T_POSE cam_poses){
  // Set the camera with more measurement as anchor frame
  int total_meas = 0;
  size_t anchor_most_meas = 0;
  size_t most_meas = 0;
  for (auto const &pair : line_feat->timestamps) {
    total_meas += (int)pair.second.size();
    if (pair.second.size() > most_meas) {
      anchor_most_meas = pair.first;
      most_meas = pair.second.size();
    }
  }
  line_feat->anchor_cam_id = anchor_most_meas;
  line_feat->anchor_clone_timestamp = line_feat->timestamps.at(line_feat->anchor_cam_id).back();
  
  std::vector <Vector6d> lines;
  Eigen::Vector4d plane0;
  Eigen::Vector4d plane1;
  // loop through to get the line result
  for (auto const &pair : line_feat->timestamps) {
    // Select the first observation as the reference frame 
    const Eigen::Matrix<double, 3, 3> &R_GtoC0 = cam_poses.at(pair.first).at(line_feat->timestamps.at(pair.first).at(0)).Rot();
    const Eigen::Matrix<double, 3, 1> &p_C0inG = cam_poses.at(pair.first).at(line_feat->timestamps.at(pair.first).at(0)).pos();
    Eigen::Vector4f Line0_uv_norm;
    Line0_uv_norm << line_feat->line_uvs_norm.at(pair.first).at(0)(0), line_feat->line_uvs_norm.at(pair.first).at(0)(1), line_feat->line_uvs_norm.at(pair.first).at(0)(2), line_feat->line_uvs_norm.at(pair.first).at(0)(3);

    Eigen::Vector3d point11, point12;
    point11 << Line0_uv_norm[0], Line0_uv_norm[1], 1;
    point12 << Line0_uv_norm[2], Line0_uv_norm[3], 1;
    Eigen::Vector3d cam;
    cam << 0, 0, 0;

    // compute the plane consist of these three points
    // Eigen::Vector4d plane0;
    if(!CompoutePlaneFromPoints(point11, point12, cam, plane0)) {
        return false;
    }

    for (size_t m = 1; m < line_feat->timestamps.at(pair.first).size(); m++) {
      // Get the position of this clone in the global
      const Eigen::Matrix<double, 3, 3> &R_GtoCi = cam_poses.at(pair.first).at(line_feat->timestamps.at(pair.first).at(m)).Rot();
      const Eigen::Matrix<double, 3, 1> &p_CiinG = cam_poses.at(pair.first).at(line_feat->timestamps.at(pair.first).at(m)).pos();
    
      // Get the trasnform matrix from current frame to the reference frame 
      Eigen::Matrix<double, 3, 3> R_C0toCi = R_GtoCi * R_GtoC0.transpose() ;
      Eigen::Matrix<double, 3, 1> p_CiinC0 = R_GtoC0 * (p_CiinG - p_C0inG);

      // Get the 2D position in the pixel coordinate 
      Eigen::Vector4f Line_uv_norm;
      Line_uv_norm << line_feat->line_uvs_norm.at(pair.first).at(m)(0), line_feat->line_uvs_norm.at(pair.first).at(m)(1), line_feat->line_uvs_norm.at(pair.first).at(m)(2), line_feat->line_uvs_norm.at(pair.first).at(m)(3);
      
      Eigen::Vector3d point31, point32;
      point31 << Line_uv_norm[0], Line_uv_norm[1], 1;
      point32 << Line_uv_norm[2], Line_uv_norm[3], 1;
      point31 = R_C0toCi.transpose() * point31 + p_CiinC0;
      point32 = R_C0toCi.transpose() * point32 + p_CiinC0;

      Vector3d cam_1 = p_CiinC0;

      // compute the plane consist of these three points
      // Eigen::Vector4d plane1;
      if(!CompoutePlaneFromPoints(point31, point32, cam_1, plane1)) {
        continue;
      }
      // compuete the Plucker coordinate of 3D line across the planes
      Vector6d line;
      if (!ComputeLineFramePlanes(plane0, plane1, line)) {
        continue;
      }
      lines.push_back(line);
    }
  }
  if (lines.empty()) {
    return false;
  }
  
  // line trangulation result (3D position of the line in the first camera frame)
  Vector6d line_result;
  Eigen::Vector3d direction_sum(0, 0, 0);
  Eigen::Vector3d normal_sum(0, 0, 0);
  double direction_norm_sum = 0; 
  double normal_norm_sum = 0;
  for (size_t i = 0; i < lines.size(); i++) {
    Vector3d normal_vector = lines[i].head(3);
    Vector3d direction_vector = lines[i].tail(3);
    direction_sum += direction_vector;
    normal_sum += normal_vector;
    direction_norm_sum += direction_vector.norm();
    normal_norm_sum += normal_vector.norm();
  }
  // line_result.head(3) = (direction_sum / direction_sum.norm());
  // line_result.tail(3) = (normal_sum / normal_sum.norm());
  line_result.head(3) = (direction_sum / direction_norm_sum);
  line_result.tail(3) = (normal_sum / lines.size());
  line_feat->line_FinA = line_result;

  // transform the line from reference camera frame to the world frame
  Eigen::Matrix<double, 3, 3> R_GtoC0 = cam_poses.at(0).at(line_feat->timestamps.at(0).at(0)).Rot();
  Eigen::Matrix<double, 3, 1> p_C0inG = cam_poses.at(0).at(line_feat->timestamps.at(0).at(0)).pos();
  Vector6d line_in_world;
  line_in_world.tail(3) = R_GtoC0.transpose() * line_result.head(3);
  line_in_world.head(3) = R_GtoC0.transpose() * line_result.tail(3) + ov_core::skew_x(p_C0inG) * R_GtoC0.transpose() * line_result.head(3);
  line_feat->line_FinG = line_in_world;

  // Set the end-points
  Vector6d endpoints;
  Matrix4d Lc;
  Vector3d nc = line_result.tail(3);
  Vector3d vc = line_result.head(3);
  Lc << ov_core::skew_x(nc), vc, -vc.transpose(), 0;
  Vector4d e1 = Lc * plane0;
  e1 = e1/e1(3);
  Vector3d pts_1(e1(0),e1(1),e1(2));
  Vector4d e2 = Lc * plane1;
  e1 = e2/e2(3);
  Vector3d pts_2(e2(0),e2(1),e2(2));
  Eigen::Vector3d p1 = R_GtoC0.transpose() * pts_1 + p_C0inG;
  Eigen::Vector3d p2 = R_GtoC0.transpose() * pts_2 + p_C0inG;
  // endpoints.head(3) = p1;
  // endpoints.tail(3) = p2;
  // Plucker_To_TwoPoints(line_in_world, endpoints, R_GtoC0, p_C0inG);
  line_feat->EndPoints = endpoints;
  return true;
}

// TODO::Optimizate the result and filt bad one
bool LineHelper::line_gaussnewton(std::shared_ptr<LineFeature> &line_feat, ID_T_POSE cam_poses){
  return true;
}

void LineHelper::Plucker_To_TwoPoints(Vector6d &plucker, Vector6d &two_points, Eigen::Matrix<double, 3, 3> &R_GtoC0, Eigen::Matrix<double, 3, 1> &p_C0inG){
  Vector3d direction = plucker.head(3);
  Vector3d normal = plucker.tail(3);
  double norm_d = direction.norm();
  if (norm_d < 1e-6) {
    std::cout << "Invalid Plücker coordinates: direction vector is zero." << std::endl;
    return;
  }

  // 计算直线上的一点 p0
  Eigen::Vector3d p0 = normal.cross(direction) / (norm_d * norm_d);
  double t = 5;
  Eigen::Vector3d p1 = p0 + t * direction;
  Eigen::Vector3d p2 = p0 - t * direction;
  // p1 = R_GtoC0.transpose() * p1 + p_C0inG;
  // p2 = R_GtoC0.transpose() * p2 + p_C0inG;
  two_points.head(3) = p1;
  two_points.tail(3) = p2;
}

void LineHelper::cleanup_lines(shared_ptr<State> state, L_Feature &lines, LDB_ptr ldb, std::vector<Vector6d> &used, shared_ptr<TrackLSD> track_line) {
  // Save all the lines features used in the update
  for (auto const &line : lines){
    if (line->triangulated) {
      used.push_back(line->EndPoints);
    } else {
      continue;
    }
  }
  // Get vector of features unused
  vector<shared_ptr<LineFeature>> vec_unused;
  vec_unused.reserve(ldb->get_internal_data().size());
  for (auto f : ldb->get_internal_data())
    vec_unused.push_back(f.second);

  // Sort based on track length
  sort(vec_unused.begin(), vec_unused.end(), feat_sort);

  // Put in the database to return
  LDB_ptr db_return = make_shared<LineFeatureDatabase>();
  copy_to_db(db_return, vec_unused);

  // This allows for measurements to be used in the future if they failed to be used this time
  // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
  auto feats_database = track_line->get_feature_database();
  feats_database->append_new_measurements(db_return);

  // Cleanup any features older than the marginalization time
  if (state->clone_window() > state->op->window_size) {
    feats_database->cleanup_measurements(state->oldest_clone_time());
  }
}

int LineHelper::n_meas(std::shared_ptr<LineFeature> feat) {
  int n_meas = 0;
  for (const auto &pair : feat->timestamps)
    n_meas += pair.second.size();
  return n_meas;
}

int LineHelper::n_meas(std::vector<std::shared_ptr<LineFeature>> vec_feat) {
  int num_meas = 0;
  for (const auto &feat : vec_feat)
    num_meas += n_meas(feat);
  return num_meas;
}

int LineHelper::n_meas(std::shared_ptr<LineFeatureDatabase> db) {
  int num_meas = 0;
  for (const auto &id_feature : db->get_internal_data())
    num_meas += n_meas(id_feature.second);
  return num_meas;
}

void LineHelper::copy_to_db(LDB_ptr ldb, shared_ptr<LineFeature> &feature) {
  for (const auto &cam_id : feature->timestamps) {
    for (const auto &mtime : cam_id.second) {
      copy_to_db(ldb, feature, cam_id.first, mtime);
    }
  }
}

void LineHelper::copy_to_db(LDB_ptr ldb, std::vector<shared_ptr<LineFeature>> &vec_feature) {
  for (auto &feature : vec_feature) {
    for (const auto &cam_id : feature->timestamps) {
      for (const auto &mtime : cam_id.second) {
        copy_to_db(ldb, feature, cam_id.first, mtime);
      }
    }
  }
}
void LineHelper::copy_to_db(LDB_ptr ldb, shared_ptr<LineFeature> &feature, unsigned long cam_id, double time) {
  // fine the uv measurments correspond to the measurement time
  auto it0 = find(feature->timestamps.at(cam_id).begin(), feature->timestamps.at(cam_id).end(), time);
  assert(it0 != feature->timestamps.at(cam_id).end());
  auto idx0 = distance(feature->timestamps.at(cam_id).begin(), it0);
  // get uv measurement
  Eigen::Vector4f line_uv = feature->line_uvs.at(cam_id).at(idx0);
  Eigen::Vector4f line_uv_n = feature->line_uvs_norm.at(cam_id).at(idx0);
  // move measurement
  ldb->update_feature(feature->featid, time, cam_id, line_uv, line_uv_n, feature->dynamic);
}

bool LineHelper::feat_sort(const shared_ptr<LineFeature> &a, const shared_ptr<LineFeature> &b) {
  size_t asize = 0;
  size_t bsize = 0;
  for (const auto &pair : a->timestamps)
    asize += pair.second.size();
  for (const auto &pair : b->timestamps)
    bsize += pair.second.size();
  return asize > bsize;
};

bool LineHelper::CompoutePlaneFromPoints(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2, const Eigen::Vector3d &point3, Eigen::Vector4d &plane) {
  // Eigen::Vector3d line12 = point2 - point1;
  // Eigen::Vector3d line13 = point3 - point1;
  // Eigen::Vector3d n = line12.cross(line13);
  // plane.head(3) = n.normalized();
  // plane(3) = - n.transpose() * point1;
  plane << (point1 - point3).cross(point2 - point3), -point3.dot(point1.cross(point2));
  return true;
}

bool LineHelper::ComputeLineFramePlanes(const Eigen::Vector4d& plane1, const Eigen::Vector4d& plane2, Vector6d &line_3d) {
  // Eigen::Vector3d n1 = plane1.head(3);
  // Eigen::Vector3d n2 = plane2.head(3);

  // double cos_theta = n1.transpose() * n2;
  // cos_theta /= (n1.norm() * n2.norm());

  // Eigen::Vector3d d = n1.cross(n2);
  // Eigen::Vector3d w = plane2(3) * n1 - plane1(3) * n2; 
  // line_3d.head(3) = d.normalized();
  // line_3d.tail(3) = w;
  Eigen::Vector3d norm1 = plane1.head(3); 
  norm1.normalize(); 
  Eigen::Vector3d norm2 = plane2.head(3); 
  norm2.normalize(); 
  double cos_theta = norm1.dot(norm2)/(norm1.norm() * norm2.norm());
  
  if(abs(cos_theta) >= 0.99) {
    return false;
  }
  Matrix4d dp = plane1 * plane2.transpose() - plane2 * plane1.transpose();
  
  line_3d << dp(0,3), dp(1,3), dp(2,3), - dp(1,2), dp(0,2), - dp(0,1);
  
  return true;
}

void LineHelper::remove_unusable_measurements(shared_ptr<State> state, L_Feature &l_feats, LDB_ptr db) {
  // Loop though the features
  for (auto feat = l_feats.begin(); feat != l_feats.end();) {
    vector<double> invalid_times;
    for (const auto &pair : (*feat)->timestamps) {
      for (const auto &meas_t : pair.second) {
        // check if this measurement is "newer" than our window. We allow extrapolation
        if (meas_t + state->cam_dt.at(pair.first)->value()(0) > state->time + 0.01) {
          // return it back to db
          copy_to_db(db, (*feat), pair.first, meas_t);
          invalid_times.push_back(meas_t);
          continue;
        }
        // check if this measurement is "older" than our window. We allow extrapolation
        if (meas_t + state->cam_dt.at(pair.first)->value()(0) < state->oldest_clone_time() - 0.01) {
          // discard the measurement as it cannot be processed
          invalid_times.push_back(meas_t);
          continue;
        }
      }
    }
    // remove invalid measurements from feature vector
    (*feat)->clean_invalid_measurements(invalid_times);

    // remove if feature has no measurements
    if (n_meas(*feat) == 0 || n_meas(*feat) == 1)
      feat = l_feats.erase(feat);
    else // Otherwise this is good
      feat++;
  }
}

Vector4d LineHelper::Plucker_to_Orth(const Vector6d& plucker) {
  Vector4d orth;
  Vector3d n = plucker.tail(3);
  Vector3d v = plucker.head(3);

  Vector3d u1 = n/n.norm();
  Vector3d u2 = v/v.norm();
  Vector3d u3 = u1.cross(u2);

  orth[0] = atan2( u2(2),u3(2) );
  orth[1] = asin( -u1(2) );
  orth[2] = atan2( u1(1),u1(0) );

  Vector2d w( n.norm(), v.norm() );
  w = w/w.norm();
  orth[3] = asin( w(1) );
  return orth;      
}

Vector6d LineHelper::Orth_to_Plucker(const Vector4d& orth) {
  Vector6d plucker;
  Vector3d theta = orth.head(3);
  double phi = orth[3];

  double s1 = sin(theta[0]);
  double c1 = cos(theta[0]);
  double s2 = sin(theta[1]);
  double c2 = cos(theta[1]);
  double s3 = sin(theta[2]);
  double c3 = cos(theta[2]);

  Matrix3d R;
  R << c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
       c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
       -s2,       s1 * c2,                  c1 * c2;
  Vector3d u1 = R.col(0);
  Vector3d u2 = R.col(1);

  double w1 = cos(phi);
  double w2 = sin(phi);

  Vector3d n = w1 * u1;
  Vector3d v = w2 * u2;

  plucker.head(3) = v;
  plucker.tail(3) = n;
  return plucker;
}

LineLinSys LineHelper::get_line_feature_jacobian_full(shared_ptr<State> state, shared_ptr<LineFeature> line_feat, LDB_ptr ldb, ID_T_POSE imu_poses, bool PLC){
  //=========================================================================
  // Linear System
  //=========================================================================
  LineLinSys linsys;

  // Compute the size of measurements for this feature and the states involved with this feature
  int total_hx = 0;
  int total_meas = n_meas(line_feat);
  unordered_map<shared_ptr<ov_type::Type>, size_t> map_hx;
  vector<double> invalid_measurements;
  std::unordered_map<size_t, std::vector<double>> timestamps = line_feat->timestamps;
  bool use_PLC = PLC;
  std::unordered_map <size_t, std::map<double, std::vector<Eigen::Vector2f>>> point_uvs = line_feat->point_uvs;

  ///////////////////////////////////////////////////////////////////////
  /////////// Future work(find why this bug happens) ////////////////////
  ///////////////////////////////////////////////////////////////////////
  if (point_uvs.size() == 0) {
    use_PLC = false;
  }
  ///////////////////////////////////////////////////////////////////////
  
  for (auto &pair : timestamps) {
    
    // Our extrinsics and intrinsics
    shared_ptr<ov_type::PoseJPL> calibration = state->cam_extrinsic.at(pair.first);
    shared_ptr<ov_type::Vec> distortion = state->cam_intrinsic.at(pair.first);
    shared_ptr<ov_type::Vec> timeoffset = state->cam_dt.at(pair.first);

    // Loop through all measurements for this specific camera
    for (auto meas_time = timestamps.at(pair.first).begin(); meas_time != timestamps.at(pair.first).end();) {
      vector<shared_ptr<ov_type::Type>> order;
      auto D = Dummy();
      if (state->get_interpolated_jacobian((*meas_time) + timeoffset->value()(0), D.R, D.p, "CAM", pair.first, D.VM, order)) {
        for (const auto &type : order) {
          if (map_hx.find(type) == map_hx.end()) {
            map_hx.insert({type, total_hx});
            linsys.Hx_order.push_back(type);
            total_hx += type->size();
          }
        }
        meas_time++;
      } else {
        // case fail, remove from list
        total_meas--;                                                    // reduce total meas
        copy_to_db(ldb, line_feat, pair.first, (*meas_time));         // copy to db
        invalid_measurements.push_back((*meas_time));                    // mark it as invalid measurement. Will be removed from feature later
        meas_time = timestamps.at(pair.first).erase(meas_time); // erase measurement from feature and move on

        // also remove the invalided Point-Line-Coupled measurements
        if (use_PLC) {
          point_uvs.at(pair.first).erase(*meas_time);
        }
      }
    }        
  }

  // If we use Point-Line-coupled feature, we need to increase the demension
  int PLC_meas = 0;
  if (use_PLC) {
    for (const auto &pair : point_uvs) {
      for (const auto &time : pair.second) {
        PLC_meas += time.second.size();
      }
    }
  }

  // remove invalid measurements from feature and put it back to db
  line_feat->clean_invalid_measurements(invalid_measurements);

  // Allocate our residual and Jacobians
  int c = 0;
  linsys.res = VectorXd::Zero(2 * total_meas + PLC_meas);
  linsys.Hf = MatrixXd::Zero((2 * total_meas + PLC_meas), 6);
  linsys.Hx = MatrixXd::Zero((2 * total_meas + PLC_meas), total_hx);
  linsys.R = MatrixXd::Identity((2 * total_meas + PLC_meas), (2 * total_meas + PLC_meas)); // This should be the result of whitening.

  // Derivative of p_FinG in respect to feature representation.
  // This only needs to be computed once and thus we pull it out of the loop
  MatrixXd intr_err_cov = state->intr_pose_cov(state->op->clone_freq, state->op->intr_order);

  // Loop through each camera for this feature
  for (auto const &pair : timestamps) {

    // Our calibration between the IMU and cami frames
    int cam_id = pair.first;
    shared_ptr<ov_type::Vec> distortion = state->cam_intrinsic.at(cam_id);
    shared_ptr<ov_type::PoseJPL> calibration = state->cam_extrinsic.at(cam_id);
    shared_ptr<ov_type::Vec> timeoffset = state->cam_dt.at(cam_id);
    Matrix3d R_ItoC = calibration->Rot();
    Vector3d p_IinC = calibration->pos();
    double dt = timeoffset->value()(0);

    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < timestamps.at(cam_id).size(); m++) {
      //=========================================================================
      // Residual
      //=========================================================================
      double tm = timestamps.at(cam_id).at(m);
      bool find_point = false;
      std::vector<Eigen::Vector2f> feature_points;
      
      if (use_PLC && point_uvs.at(cam_id).find(tm) != point_uvs.at(cam_id).end())
      {
        find_point = true;
        feature_points = point_uvs.at(cam_id).at(tm);
      }
      
      Matrix3d R_GtoI = imu_poses.at(cam_id).at(tm).Rot();
      Vector3d p_IinG = imu_poses.at(cam_id).at(tm).pos();      
      
      // Convert line feature from world frame to the IMU frame
      MatrixXd G_to_I = MatrixXd::Zero(6, 6);
      G_to_I.block(0, 0, 3, 3) = R_GtoI;
      G_to_I.block(0, 3, 3, 3) = -R_GtoI * ov_core::skew_x(p_IinG);
      G_to_I.block(3, 3, 3, 3) = R_GtoI;
      Vector6d p_FinI = G_to_I * line_feat->line_FinG;   

      // Project the current feature into the current frame of reference
      MatrixXd I_to_C = MatrixXd::Zero(6, 6);
      I_to_C.block(0, 0, 3, 3) = R_ItoC;
      I_to_C.block(0, 3, 3, 3) = ov_core::skew_x(p_IinC) * R_ItoC;
      I_to_C.block(3, 3, 3, 3) = R_ItoC;
      Vector6d p_FinC = I_to_C * p_FinI;

      // Distort the 3D line
      Vector3d line_dist;
      Eigen::MatrixXd intrinsic = state->cam_intrinsic_model.at(cam_id)->get_value();
      Matrix3d K;
      K << intrinsic(1), 0, 0, 0, intrinsic(0), 0, -intrinsic(1) * intrinsic(2), -intrinsic(0) * intrinsic(3), intrinsic(0) * intrinsic(1);
      line_dist = K * p_FinC.head(3);
      // line_dist = p_FinC.head(3);
      // Line residual
      Vector3d uv_s, uv_e;
      uv_s << (double)line_feat->line_uvs.at(cam_id).at(m)(0), (double)line_feat->line_uvs.at(cam_id).at(m)(1), 1;
      uv_e << (double)line_feat->line_uvs.at(cam_id).at(m)(2), (double)line_feat->line_uvs.at(cam_id).at(m)(3), 1;
      // uv_s << (double)line_feat->line_uvs_norm.at(cam_id).at(m)(0), (double)line_feat->line_uvs_norm.at(cam_id).at(m)(1), 1;
      // uv_e << (double)line_feat->line_uvs_norm.at(cam_id).at(m)(2), (double)line_feat->line_uvs_norm.at(cam_id).at(m)(3), 1;
      Eigen::VectorXd error_s, error_e;
      error_s = (uv_s.transpose() * line_dist) / (sqrt(line_dist(0) * line_dist(0) + line_dist(1) * line_dist(1)));
      error_e = (uv_e.transpose() * line_dist) / (sqrt(line_dist(0) * line_dist(0) + line_dist(1) * line_dist(1)));
      Vector2d res;
      res << error_s(0), error_e(0);
      linsys.res.block(2 * c, 0, 2, 1) = res;
      
      // point-line-coupled residual
      if (use_PLC && find_point) {
        int i = 0;
        for (auto p = point_uvs.at(cam_id).at(tm).begin(); p != point_uvs.at(cam_id).at(tm).end();) {
          Vector3d p_uv_s;
          p_uv_s << (double)(*p)(0), (double)(*p)(1), 1;
          Eigen::VectorXd error = (p_uv_s.transpose() * line_dist) / (sqrt(line_dist(0) * line_dist(0) + line_dist(1) * line_dist(1)));
          linsys.res.block((2 * c + i), 0, 1, 1) = error;
          i++;
          p++;
        }
      }
      //=========================================================================
      // Jacobian
      //=========================================================================
      // Get Jacobian of interpolated pose in respect to the state
      vector<MatrixXd> dTdx;
      vector<shared_ptr<ov_type::Type>> order;
      state->get_interpolated_jacobian(tm + dt, R_GtoI, p_IinG, "CAM", cam_id, dTdx, order);
      
      // Observation error in respective to line
      // Without PLC residual, the dimention should be 2*3
      // with PLC residual, the dimension should be (2 + n) * 3
      MatrixXd dz_l;

      int n_p = feature_points.size();
      if (use_PLC && find_point) {
        dz_l = MatrixXd::Identity(2 + n_p, 3);
        double ln_2 = line_dist(0) * line_dist(0) + line_dist(1) + line_dist(1);

        dz_l(0, 0) = uv_s(0) - (line_dist(0) * uv_s.transpose() * line_dist)(0) / (ln_2);
        dz_l(0, 1) = uv_s(1) - (line_dist(1) * uv_s.transpose() * line_dist)(0) / (ln_2);
        dz_l(1, 0) = uv_e(0) - (line_dist(0) * uv_e.transpose() * line_dist)(0) / (ln_2);
        dz_l(1, 1) = uv_e(1) - (line_dist(1) * uv_e.transpose() * line_dist)(0) / (ln_2);
        for (int i = 0; i < n_p; i++) {
          Vector3d p_uv_s;
          p_uv_s << (double)feature_points[i](0), (double)feature_points[i](1), 1;
          dz_l(2 + i, 0) = p_uv_s(0) - (line_dist(0) * p_uv_s.transpose() * line_dist)(0) / (ln_2);
          dz_l(2 + i, 1) = p_uv_s(1) - (line_dist(0) * p_uv_s.transpose() * line_dist)(0) / (ln_2);
        }
        dz_l *= 1 / sqrt(ln_2);
      } else {
        dz_l = MatrixXd::Identity(2, 3);
        double ln_2 = line_dist(0) * line_dist(0) + line_dist(1) + line_dist(1);

        dz_l(0, 0) = uv_s(0) - (line_dist(0) * uv_s.transpose() * line_dist)(0) / (ln_2);
        dz_l(0, 1) = uv_s(1) - (line_dist(1) * uv_s.transpose() * line_dist)(0) / (ln_2);
        dz_l(1, 0) = uv_e(0) - (line_dist(0) * uv_e.transpose() * line_dist)(0) / (ln_2);
        dz_l(1, 1) = uv_e(1) - (line_dist(1) * uv_e.transpose() * line_dist)(0) / (ln_2);
        dz_l *= 1 / sqrt(ln_2);
      }

      // Compute Jacobians in respect to normalized image coordinates 
      MatrixXd dl_ln = MatrixXd::Zero(3, 6);
      dl_ln.block(0, 0, 3, 3) = K;

      // Nomalized observation in respect to imu frame 
      MatrixXd dln_li = MatrixXd::Zero(6, 6);
      dln_li = I_to_C;

      // Jacobians in respect to imu pose
      MatrixXd dli_dI = MatrixXd::Zero(6, 6);
      // Rotation part
      dli_dI.block(0, 0, 3, 3) = ov_core::skew_x(R_GtoI * (line_feat->line_FinG.head(3) - ov_core::skew_x(p_IinG) * line_feat->line_FinG.tail(3)));
      dli_dI.block(3, 0, 3, 3) = ov_core::skew_x(R_GtoI * line_feat->line_FinG.tail(3));
      // Translation part
      dli_dI.block(0, 3, 3, 3) = R_GtoI * ov_core::skew_x(line_feat->line_FinG.tail(3));

      // Jacobians in respect to line feature in world frame
      MatrixXd dli_dlg = MatrixXd::Zero(6, 6);
      dli_dlg = G_to_I;

      // Precompute some matrices
      MatrixXd dz_li = dz_l * dl_ln * dln_li;

      // CHAINRULE: get the total Jacobian of in respect to feature and interpolated pose
      Eigen::MatrixXd HI = dz_l * dl_ln * dln_li * dli_dI;    

      //=========================================================================
      // Find the LLT of R inverse so that we can efficiently whiten the noise
      //=========================================================================
      // default line projection covariance
      Eigen::MatrixXd R;
      if (use_PLC && find_point) {
        R = MatrixXd::Identity(2 + n_p, 2 + n_p);
      } else {
        R = pow(state->op->cam->sigma_pix, 2) * Matrix2d::Identity();
      }
      // append interpolated covariance
      if (!state->have_clone(tm + dt)) {
        if (state->op->use_pol_cov) {
          R.noalias() += HI * intr_err_cov * HI.transpose();
        } else if (state->op->use_imu_cov) {
          MatrixXd H_cpi = MatrixXd::Zero(6, 6); // pose to gamma and alpha
          H_cpi.block(0, 0, 3, 3) = Matrix3d::Identity();
          H_cpi.block(3, 3, 3, 3) = state->clones.at(state->cpis.at(tm + dt).clone_t)->Rot_fej().transpose();

          MatrixXd H_ = HI * H_cpi;
          R.noalias() += H_ * state->cpis.at(tm + dt).Q * H_.transpose() * state->op->intr_err.mlt;
        }        
      }

      // Find the LLT of R inverse
      // llt() means Cholesky (used for solve Ax = b problem, more stable and efficient)
      if (use_PLC && find_point) {
        MatrixXd R_llt = R.llt().matrixL();
        MatrixXd R_llt_inv = R_llt.llt().solve(MatrixXd::Identity(2 + n_p, 2 + n_p));

        // Whiten residue with the measurement noise
        linsys.res.block(2 * c, 0, 2 + n_p, 1) = R_llt_inv * linsys.res.block(2 * c, 0, 2 + n_p, 1);
        dz_li = R_llt_inv * dz_li;

      } else {
        MatrixXd R_llt = R.llt().matrixL();
        MatrixXd R_llt_inv = R_llt.llt().solve(MatrixXd::Identity(2, 2));

        // Whiten residue with the measurement noise
        linsys.res.block(2 * c, 0, 2, 1) = R_llt_inv * linsys.res.block(2 * c, 0, 2, 1);
      
        // Whiten the jacobians.
        dz_li = R_llt_inv * dz_li;
      }
      
      //=========================================================================
      // Jacobian continues
      //========================================================================= 
      if (use_PLC && find_point) {
        linsys.Hf.block(2 * c, 0, 2 + n_p, linsys.Hf.cols()).noalias() += dz_li * dli_dlg;
        for (int i = 0; i < (int)dTdx.size(); i++) {
          assert(map_hx.find(order.at(i)) != map_hx.end());
          linsys.Hx.block(2 * c, map_hx.at(order.at(i)), 2 + n_p, order.at(i)->size()).noalias() += dz_li * dli_dI * dTdx.at(i);
        }
      } else {
        linsys.Hf.block(2 * c, 0, 2, linsys.Hf.cols()).noalias() += dz_li * dli_dlg;
        // CHAINRULE: get state clone Jacobian. This also adds timeoffset jacobian
        for (int i = 0; i < (int)dTdx.size(); i++) {
          assert(map_hx.find(order.at(i)) != map_hx.end());
          linsys.Hx.block(2 * c, map_hx.at(order.at(i)), 2, order.at(i)->size()).noalias() += dz_li * dli_dI * dTdx.at(i);
        }
      }
      // Move the Jacobian and residual index forward
      c++;
    }
  }
  return linsys;
}

void LineHelper::Vanishing_Points(shared_ptr<State> state, std::vector<Eigen::Vector2d> &vanishing_points, const ov_core::CameraData &message) {
  int cam_id = message.sensor_ids.at(0);
  // double dt = state->cam_dt.at(cam_id)->value()(0);
  // double timestamps = state->newest_clone_time();

  Matrix3d R_ItoC = state->cam_extrinsic.at(cam_id)->Rot();
  Vector3d p_IinC = state->cam_extrinsic.at(cam_id)->pos();

  // Compute imu pose
  Matrix3d R_GtoI;
  Vector3d p_IinG;
  R_GtoI = state->imu->Rot();
  p_IinG = state->imu->pos();
  Eigen::Vector3d vp_x = R_ItoC * Eigen::Vector3d(1, 0, 0); 
  Eigen::Vector3d vp_y = R_ItoC * Eigen::Vector3d(0, 1, 0);
  Eigen::Vector3d vp_z = R_ItoC * Eigen::Vector3d(0, 0, 1);

  Vector2d vp_x_dist, vp_y_dist, vp_z_dist;
  Eigen::MatrixXd cam_d = state->cam_intrinsic_model.at(cam_id)->get_value();
  Distort(vp_x, cam_d, vp_x_dist);
  Distort(vp_y, cam_d, vp_y_dist);
  Distort(vp_z, cam_d, vp_z_dist);

  vanishing_points[0] = vp_x_dist;
  vanishing_points[1] = vp_y_dist;
  vanishing_points[2] = vp_z_dist;
  double y = vp_z_dist(1);
  y *= 1000;
  vp_z_dist(1) = y;
  vanishing_points[2] = vp_z_dist;
}

void LineHelper::Distort(Eigen::Vector3d &vp_x, Eigen::MatrixXd &cam_d, Vector2d &vp_x_dist) {
    // // Calculate distorted coordinates for fisheye
    // double r = std::sqrt(vp_x(0) * vp_x(0) + vp_x(1) * vp_x(1));
    // double theta = std::atan(r);
    // double theta_d = theta + cam_d(4) * std::pow(theta, 3) + cam_d(5) * std::pow(theta, 5) + cam_d(6) * std::pow(theta, 7) +
    //                  cam_d(7) * std::pow(theta, 9);

    // // Handle when r is small (meaning our xy is near the camera center)
    // double inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
    // double cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;
    
    // // Calculate distorted coordinates for fisheye
    // Eigen::Vector2f uv_dist;
    // double x1 = vp_x(0) * cdist;
    // double y1 = vp_x(1) * cdist;
    // vp_x_dist(0) = (float)(cam_d(0) * x1 + cam_d(2));
    // vp_x_dist(1) = (float)(cam_d(1) * y1 + cam_d(3));

    // Calculate distorted coordinates for radial
    double r = std::sqrt(vp_x(0) * vp_x(0) + vp_x(1) * vp_x(1));
    double r_2 = r * r;
    double r_4 = r_2 * r_2;
    double x1 = vp_x(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 2 * cam_d(6) * vp_x(0) * vp_x(1) +
                cam_d(7) * (r_2 + 2 * vp_x(0) * vp_x(0));
    double y1 = vp_x(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + cam_d(6) * (r_2 + 2 * vp_x(1) * vp_x(1)) +
                2 * cam_d(7) * vp_x(0) * vp_x(1);

    // Return the distorted point
    vp_x_dist(0) = (float)(cam_d(0) * x1 + cam_d(2));
    vp_x_dist(1) = (float)(cam_d(1) * y1 + cam_d(3));
}
