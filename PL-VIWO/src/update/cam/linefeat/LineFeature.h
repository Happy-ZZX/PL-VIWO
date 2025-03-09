#ifndef VIW_LINEFEATURE_H
#define VIW_LINEFEATURE_H

#include <Eigen/Eigen>
#include <unordered_map>
#include <vector>

// Eigen type
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;

namespace viw {

/**
 * @brief Line feature class used to collect measurements
 * 
 * This line feature class allows for holding of all tracking information for a given line feature.
 * Each feature has a unique ID assigned to it, and should have a set of feature tracks alongside it.
 * See the FeatureDatabase class for details on how we load information into this, and how we delete features.
 */
class LineFeature {
public:
  /// Unique ID of this line feature 
  size_t featid;
  
  /// If this feature should be deleted
  bool to_delete;
  
  /// If this feature fails in the Chi2_test
  bool dynamic = false; 
  
  /// If this feature is stereo
  bool stereo_feature = true;

  /// If this line is triangulated
  bool triangulated = false;

  /// UV coordinate of the start and end points that this line feature has been seen from (mapped by camera ID)
  std::unordered_map <size_t, std::vector<Eigen::VectorXf>> line_uvs;

  /// UV normalized coordinate of the start and end points that this line feature has been seen from (mapped by camera ID)
  std::unordered_map <size_t, std::vector<Eigen::VectorXf>> line_uvs_norm;

  /// Timestamps of each UV measurement (mapped by camera ID)
  std::unordered_map<size_t, std::vector<double>> timestamps;

  /// Feature points ID assigned with line
  std::vector<int> points;

  /// 2D Feature points assigned with line 
  std::unordered_map <size_t, std::map<double, std::vector<Eigen::Vector2f>>> point_uvs;

  /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
  int anchor_cam_id = -1;

  /// Timestamp of anchor clone
  double anchor_clone_timestamp;

  /// Line Paralle (0 means no paralle, 1 means x axis, 2 y axis and 3 z axis)
  int D = 0;

  /// Triangulated Plucker coodinate of this feature, in the anchor frame (First Observation Frame)
  /// First 3 number represent the normal vector
  /// Last  3 number represent the direction vector 
  Vector6d line_FinA;

  /// Triangulated Plucker coodinate of this feature, in the global frame
  /// First 3 number represent the normal vector
  /// Last  3 number represent the direction vector 
  Vector6d line_FinG;
  
  /// 3D endpoints of this line (For dispaly)
  Vector6d EndPoints;
  
  /// The first observation camera pose
  Eigen::Matrix<double, 3, 3> R_GtoC;
  Eigen::Matrix<double, 3, 1> p_CinG;

  /**
   * @brief Remove measurements that do not occur at passed timestamps.
   *
   * Given a series of valid timestamps, this will remove all measurements that have not occurred at these times.
   * This would normally be used to ensure that the measurements that we have occur at our clone times.
   *
   * @param valid_times Vector of timestamps that our measurements must occur at
   */
  void clean_old_measurements(const std::vector<double> &valid_times);

  /**
   * @brief Remove measurements that occur at the invalid timestamps
   *
   * Given a series of invalid timestamps, this will remove all measurements that have occurred at these times.
   *
   * @param invalid_times Vector of timestamps that our measurements should not
   */
  void clean_invalid_measurements(const std::vector<double> &invalid_times);

  /**
   * @brief Remove measurements that are older then the specified timestamp.
   *
   * Given a valid timestamp, this will remove all measurements that have occured earlier then this.
   *
   * @param timestamp Timestamps that our measurements must occur after
   */
  void clean_older_measurements(double timestamp);   
};
} // namespace viw

#endif /* VIW_LINEFEATURE_H */