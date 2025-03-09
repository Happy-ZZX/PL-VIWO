#ifndef VIW_UPDATERCAMERA_H
#define VIW_UPDATERCAMERA_H
#include <Eigen/Eigen>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <queue>
#include <vector>

using namespace std;
namespace ov_core{
class Feature;
class FeatureDatabase;
class TrackBase;
struct CameraData;
} //namespace ov_core

namespace viw{
class State;
class UpdaterStatistics;
class LineFeature;
class LineFeatureDatabase;
struct CamSimData;
struct TimeChecker;
struct TrackLSD;

typedef vector<shared_ptr<ov_core::Feature>> V_Feature;
typedef shared_ptr<ov_core::FeatureDatabase> DB_ptr;

class UpdaterCamera {
public:
  /// Camera updater
  UpdaterCamera(shared_ptr<State> state);

  /// get real camera measurement
  void feed_measurement(const ov_core::CameraData &camdata);

  /// check available measurements and try update
  void try_update(int cam_id);

  /// return <MSCKF> features used in last update for visualization
  vector<Eigen::Vector3d> get_used_msckf();

  /// return  line features used in last update for visualization
  vector<Eigen::Matrix<double, 6, 1>> get_used_lines();

  /// return the trackFEATS (extracted features)
  map<int, shared_ptr<ov_core::TrackBase>> get_trackFEATS() {return trackFEATS;}

  /// return images used in last update for visualization
  cv::Mat get_track_img(int cam_id);

  /// Chi information
  map<int, shared_ptr<UpdaterStatistics>> Chi;

  /// measurement time information
  map<int, deque<double>> t_hist;

private:
  friend class Initializer;
  friend class Dynamic_Initializer;
  friend class ZuptUpdater;

  /// marginalize SLAM features that are lost tracking
  void marginalize_slam_features(const ov_core::CameraData &camdata);

  /// perform MSCKF update
  void msckf_update(V_Feature &vec_features, DB_ptr db_unused);

  /// perform SLAM update
  void slam_update(V_Feature &vec_features, DB_ptr db_unused);

  /// perform SLAM feature initialization
  void slam_init(V_Feature &vec_features, DB_ptr db_unused);
  
  /// perform Line feature update
  void lines_update (vector<shared_ptr<LineFeature>> &line_features, shared_ptr <LineFeatureDatabase> lbd_unused);

  // Good features that where used in the last update (used in visualization)
  vector<Eigen::Vector3d> msckf_used;

  // Good line feature that used in the last update (used in visualization)
  vector<Eigen::Matrix<double, 6, 1>> lines_used;

  /// Complete history of our feature tracks
  map<int, DB_ptr> trackDATABASE;

  /// Our sparse feature tracker (klt or descriptor)
  map<int, shared_ptr<ov_core::TrackBase>> trackFEATS;
  
  /// Our line feature tracker (LSD)
  map<int, shared_ptr<TrackLSD>> trackLSDS;

  /// Timing record
  std::shared_ptr<TimeChecker> tc;

  /// State
  shared_ptr<State> state;

  /// points that used in the sliding window
  DB_ptr point_used;
};
} // namespace viw
#endif // VIW_UPDATERCAMERA_H