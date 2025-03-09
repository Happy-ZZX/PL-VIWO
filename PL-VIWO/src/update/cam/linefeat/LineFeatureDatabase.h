#ifndef VIW_LINE_FEATURE_DATABASE_H
#define VIW_LINE_FEATURE_DATABASE_H

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
using namespace std;

namespace viw {

class LineFeature;

/**
 * @brief Database containing line features we are currently tracking
 */
class LineFeatureDatabase {

public:
  /**
   * @brief Default constructor
   */
  LineFeatureDatabase() {}
  
  /**
   * @brief Get a specified line feature 
   * @param id What line feature we want to get
   * @param remove Set to true if you want to remove the feature from the database (you will need to handle the freeing of memory)
   * @return Either a feature object, or null if it is not in the database.
   */
  std::shared_ptr<LineFeature> get_feature(size_t id, bool remove = false);

  /**
   * @brief Get a specified line feature clone (pointer is thread safe)
   * @param id What line feature we want to get
   * @param feat Feature with data in it
   * @return True if the feature was found
   */
  bool get_feature_clone(size_t id, LineFeature &feat);

  /**
   * @brief Update a feature object
   * @param id ID of the line feature we will update
   * @param timestamp time that this measurement occured at
   * @param cam_id which camera this measurement was from
   * @param line raw image coordinate
   * @param line_n undistorted/normalized image coordinate
   *
   * This will update a given feature based on the passed ID it has.
   * It will create a new feature, if it is an ID that we have not seen before.
   */
  void update_feature(size_t id, double timestamp, size_t cam_id, Eigen::Vector4f line, Eigen::Vector4f line_n, std::map<int, double> points_line, 
                        std::vector<Eigen::Vector2f> points, int D);

  void update_feature(size_t id, double timestamp, size_t cam_id, Eigen::Vector4f line, Eigen::Vector4f line_n, bool chi_test);

  void update_feature_stereo(size_t id, double timestamp, size_t cam_id, Eigen::Vector4f line, Eigen::Vector4f line_n, bool stereo_feature);
  
  /**
   * @brief This function will delete all feature measurements that are older then the specified timestamp
   */
  void cleanup_measurements(double timestamp);

  /**
   * @brief This function will delete all features that have been used up.
   *
   * If a feature was unable to be used, it will still remain since it will not have a delete flag set
   */
  void cleanup();

  /**
   * @brief Returns the size of the feature database
   */
  size_t size() {
    std::lock_guard<std::mutex> lck(line_mtx);
    return features_idlookup.size();
  }

  /**
   * @brief Returns the internal data (should not normally be used)
   */
  std::unordered_map<size_t, std::shared_ptr<LineFeature>> get_internal_data() {
    std::lock_guard<std::mutex> lck(line_mtx);
    return features_idlookup;
  }

  /**
   * @brief Gets the oldest time in the database
   */
  double get_oldest_timestamp();

  /**
   * @brief Will update the passed database with this database's latest feature information.
   */
  void append_new_measurements(const std::shared_ptr<LineFeatureDatabase> &database);

protected:
  /// Mutex lock for our map
  std::mutex line_mtx;

  /// Our lookup array that allow use to query based on ID
  std::unordered_map<size_t, std::shared_ptr<LineFeature>> features_idlookup;
};

} //namespace viw

#endif /* VIW_LINE_FEATURE_DATABASE_H */