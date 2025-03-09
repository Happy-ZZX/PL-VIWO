#ifndef VIW_UPDATER_LINEHELPER_H
#define VIW_UPDATER_LINEHELPER_H

#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include "LineFeatureDatabase.h"
#include "LineFeature.h"
#include "types/Vec.h"
#include "feat/FeatureInitializer.h"
#include <ceres/ceres.h>

using namespace std;
using namespace Eigen;

namespace ov_core {
struct CameraData;
class FeatureDatabase;
class Feature;
}

namespace ov_type {
class PoseJPL;
class IMU;
}

namespace viw {
class State;
struct TrackLSD;
struct LineLinSys;

typedef shared_ptr<State> State_ptr;
typedef std::vector<shared_ptr<LineFeature>> L_Feature;
typedef std::shared_ptr<LineFeatureDatabase> LDB_ptr;
typedef unordered_map<size_t, unordered_map<double, ov_core::FeatureInitializer::ClonePose>> ID_T_POSE;
typedef shared_ptr<ov_core::FeatureDatabase> DB_ptr;

/**
 * @brief Class that has helper functions for the line processor.
 *
 * Can compute the Jacobian for a single feature representation.
 * This will create the Jacobian based on what representation our state is in.
 * If we are using the anchor representation then we also have additional Jacobians in respect to the anchor state.
 * Also has functions such as nullspace projection and full jacobian construction.
 * For derivations look at @ref update-feat page which has detailed equations.
 *
 */
class LineHelper {
public: 
  /// Return line features can be used in update
  static void get_line_features(State_ptr state, L_Feature &msckf, LDB_ptr ldb_feats, deque<double> t_hist, DB_ptr point_database);
  
  /// find the feature that older than the timestamp
  static void features_not_containing_newer(shared_ptr<State> state, LDB_ptr ldb, double timestamp, L_Feature &feat_found);

  static void features_containing_older(State_ptr state, LDB_ptr ldb, double timestamp, L_Feature &feat_found);
  
  static bool line_triangulation(std::shared_ptr<LineFeature> &line_feat, ID_T_POSE imu_poses, ID_T_POSE cam_poses, bool optimization_refine, DB_ptr &point_database);
  static bool line_triangulation_from_points(std::shared_ptr<LineFeature> &line_feat, DB_ptr &database);
  static bool line_triangulation_from_points_and_direction(std::shared_ptr<LineFeature> &line_feat, DB_ptr &database, ID_T_POSE imu_poses);
  static bool line_single_triangulation(std::shared_ptr<LineFeature> &line_feat, ID_T_POSE cam_poses);
  static bool line_gaussnewton(std::shared_ptr<LineFeature> &line_feat, ID_T_POSE cam_poses);

  static void cleanup_lines(shared_ptr<State> state, L_Feature &lines, LDB_ptr ldb, std::vector<Vector6d> &used, shared_ptr<TrackLSD> track_line);

  static void Vanishing_Points(shared_ptr<State> state, std::vector<Eigen::Vector2d> &vanishing_points, const ov_core::CameraData &message);

  static void Distort(Eigen::Vector3d &vp_x, Eigen::MatrixXd &cam_d, Vector2d &vp_x_dist);

  /// copy feature information to the database
  static void copy_to_db(LDB_ptr ldb, shared_ptr<LineFeature> &feature);
  static void copy_to_db(LDB_ptr ldb, std::vector<shared_ptr<LineFeature>> &vec_feature);
  static void copy_to_db(LDB_ptr ldb, shared_ptr<LineFeature> &feature, unsigned long cam_id, double time);

  /// return number of measurements in the object
  static int n_meas(std::shared_ptr<LineFeature> feat);
  static int n_meas(std::vector<std::shared_ptr<LineFeature>> vec_feat);
  static int n_meas(std::shared_ptr<LineFeatureDatabase> db);  

  /// get set of IMU or CAM poses that correspond to the feature measurement times
  static void get_imu_poses(shared_ptr<State> state, L_Feature &l_feats, LDB_ptr ldb, ID_T_POSE &imu_poses);
  static void get_imu_poses(shared_ptr<State> state, shared_ptr<LineFeature> feat, LDB_ptr ldb, ID_T_POSE &imu_poses);
  static void get_cam_poses(shared_ptr<State> state, ID_T_POSE &imu_poses, ID_T_POSE &cam_poses);

  static bool feat_sort(const shared_ptr<LineFeature> &a, const shared_ptr<LineFeature> &b);

  static bool CompoutePlaneFromPoints(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2, const Eigen::Vector3d &point3, Eigen::Vector4d &plane);

  static bool ComputeLineFramePlanes(const Eigen::Vector4d& plane1, const Eigen::Vector4d& plane2, Vector6d &line_3d);

  static void remove_unusable_measurements(shared_ptr<State> state, L_Feature &l_feats, LDB_ptr db);

  static LineLinSys get_line_feature_jacobian_full(shared_ptr<State> state, shared_ptr<LineFeature> line_feat, LDB_ptr ldb, ID_T_POSE imu_poses, bool PLC);

  // Line representation transform 
  static Vector4d Plucker_to_Orth(const Vector6d& plucker);
  static Vector6d Orth_to_Plucker(const Vector4d& orth);
  static void Plucker_To_TwoPoints(Vector6d &plucker, Vector6d &two_points, Eigen::Matrix<double, 3, 3> &R_GtoC0, Eigen::Matrix<double, 3, 1> &p_C0inG);
  
};

struct OptimizeMomentResidual {
    OptimizeMomentResidual(const Vector3d &point, const Vector3d &direction)
        : point_(point), direction_(direction) {}

    template <typename T>
    bool operator()(const T *const m, T *residual) const {
        Eigen::Matrix<T, 3, 1> d_vec(T(direction_(0)), T(direction_(1)), T(direction_(2)));
        Eigen::Matrix<T, 3, 1> p_vec(T(point_(0)), T(point_(1)), T(point_(2)));
        Eigen::Matrix<T, 3, 1> m_vec(m[0], m[1], m[2]);

        // Compute residual: d x p - m
        Eigen::Matrix<T, 3, 1> cross = d_vec.cross(p_vec);
        Eigen::Matrix<T, 3, 1> diff = cross - m_vec;

        // Scale residual by sqrt(weight)
        residual[0] = diff(0);
        residual[1] = diff(1);
        residual[2] = diff(2);

        return true;
    }

private:
    const Vector3d point_;
    const Vector3d direction_;
};

} // namespace viw
#endif // VIW_UPDATER_LINEHELPER_H