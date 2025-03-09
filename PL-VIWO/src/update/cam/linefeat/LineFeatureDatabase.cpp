
#include "LineFeatureDatabase.h"

#include "LineFeature.h"
#include "utils/print.h"

using namespace viw;

std::shared_ptr<LineFeature> LineFeatureDatabase::get_feature(size_t id, bool remove) {
  std::lock_guard<std::mutex> lck(line_mtx);
  if (features_idlookup.find(id) != features_idlookup.end()) {
    std::shared_ptr<LineFeature> temp = features_idlookup.at(id);
    if (remove)
      features_idlookup.erase(id);
    return temp;
  } else {
    return nullptr;
  }
}

bool LineFeatureDatabase::get_feature_clone(size_t id, LineFeature &feat) {
  std::lock_guard<std::mutex> lck(line_mtx);
  if (features_idlookup.find(id) == features_idlookup.end())
    return false;
  // TODO: should probably have a copy constructor function in feature class
  std::shared_ptr<LineFeature> temp = features_idlookup.at(id);
  feat.featid = temp->featid;
  feat.to_delete = temp->to_delete;
  feat.dynamic = temp->dynamic;
  feat.line_uvs = temp->line_uvs;
  feat.line_uvs_norm = temp->line_uvs_norm;
  feat.timestamps = temp->timestamps;
  feat.anchor_cam_id = temp->anchor_cam_id;
  feat.anchor_clone_timestamp = temp->anchor_clone_timestamp;
  feat.line_FinA = temp->line_FinA;
  feat.line_FinG = temp->line_FinG;
  return true;
}

void LineFeatureDatabase::update_feature(size_t id, double timestamp, size_t cam_id, Eigen::Vector4f line, Eigen::Vector4f line_n, 
                                          std::map<int, double> points_line, std::vector<Eigen::Vector2f> points, int D) {

  // Find this feature using the ID lookup
  std::lock_guard<std::mutex> lck(line_mtx);
  if (features_idlookup.find(id) != features_idlookup.end()) {
    // Get our feature
    std::shared_ptr<LineFeature> feat = features_idlookup.at(id);
    // Append this new information to it!
    feat->line_uvs[cam_id].push_back(line);
    feat->line_uvs_norm[cam_id].push_back(line_n);
    feat->timestamps[cam_id].push_back(timestamp);
    for (auto& point : points_line) {
      feat->points.push_back(point.first);
    }
    feat->point_uvs[cam_id][timestamp] = points;
    return;
  }

  // Debug info
  // PRINT_DEBUG("featdb - adding new feature %d",(int)id);

  // Else we have not found the feature, so lets make it be a new one!
  std::shared_ptr<LineFeature> feat = std::make_shared<LineFeature>();
  feat->featid = id;
  feat->D = D;
  feat->line_uvs[cam_id].push_back(line);
  feat->line_uvs_norm[cam_id].push_back(line_n);
  feat->timestamps[cam_id].push_back(timestamp);
  for (auto& point : points_line) {
      feat->points.push_back(point.first);
  }
  feat->point_uvs[cam_id][timestamp] = points;
  // Append this new feature into our database
  features_idlookup[id] = feat;
  return;
}

void LineFeatureDatabase::update_feature(size_t id, double timestamp, size_t cam_id, Eigen::Vector4f line, Eigen::Vector4f line_n, bool dynamic) {

  // Find this feature using the ID lookup
  std::lock_guard<std::mutex> lck(line_mtx);
  if (features_idlookup.find(id) != features_idlookup.end()) {
    // Get our feature
    std::shared_ptr<LineFeature> feat = features_idlookup.at(id);
    // Append this new information to it!
    feat->line_uvs[cam_id].push_back(line);
    feat->line_uvs_norm[cam_id].push_back(line_n);
    feat->timestamps[cam_id].push_back(timestamp);
    feat->dynamic = dynamic;
    return;
  }

  // Else we have not found the feature, so lets make it be a new one!
  std::shared_ptr<LineFeature> feat = std::make_shared<LineFeature>();
  feat->featid = id;
  feat->line_uvs[cam_id].push_back(line);
  feat->line_uvs_norm[cam_id].push_back(line_n);
  feat->timestamps[cam_id].push_back(timestamp);
  feat->dynamic = dynamic;
  // Append this new feature into our database
  features_idlookup[id] = feat;
  return;
}

void LineFeatureDatabase::cleanup() {
  // Loop through all features
  // int sizebefore = (int)features_idlookup.size();
  std::lock_guard<std::mutex> lck(line_mtx);
  for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    // If delete flag is set, then delete it
    if ((*it).second->to_delete) {
      features_idlookup.erase(it++);
    } else {
      it++;
    }
  }
  // PRINT_DEBUG("feat db = %d -> %d\n", sizebefore, (int)features_idlookup.size() << std::endl;
}

double LineFeatureDatabase::get_oldest_timestamp() {
  std::lock_guard<std::mutex> lck(line_mtx);
  double oldest_time = -1;
  for (auto const &feat : features_idlookup) {
    for (auto const &camtimepair : feat.second->timestamps) {
      if (!camtimepair.second.empty() && (oldest_time == -1 || oldest_time > camtimepair.second.at(0))) {
        oldest_time = camtimepair.second.at(0);
      }
    }
  }
  return oldest_time;
}

void LineFeatureDatabase::append_new_measurements(const std::shared_ptr<LineFeatureDatabase> &database) {
  std::lock_guard<std::mutex> lck(line_mtx);

  // Loop through the other database's internal database
  for (const auto &feat : database->get_internal_data()) {
    if (features_idlookup.find(feat.first) != features_idlookup.end()) {
            // For this feature, now try to append the new measurement data
      std::shared_ptr<LineFeature> temp = features_idlookup.at(feat.first);
      for (const auto &times : feat.second->timestamps) {
        // Append the whole camera vector is not seen
        // Otherwise need to loop through each and append
        size_t cam_id = times.first;
        if (temp->timestamps.find(cam_id) == temp->timestamps.end()) {
          temp->timestamps[cam_id] = feat.second->timestamps.at(cam_id);
          temp->line_uvs[cam_id] = feat.second->line_uvs.at(cam_id);
          temp->line_uvs_norm[cam_id] = feat.second->line_uvs_norm.at(cam_id);
        } else {
          auto temp_times = temp->timestamps.at(cam_id);
          for (size_t i = 0; i < feat.second->timestamps.at(cam_id).size(); i++) {
            double time_to_find = feat.second->timestamps.at(cam_id).at(i);
            if (std::find(temp_times.begin(), temp_times.end(), time_to_find) == temp_times.end()) {
              temp->timestamps.at(cam_id).push_back(feat.second->timestamps.at(cam_id).at(i));
              temp->line_uvs.at(cam_id).push_back(feat.second->line_uvs.at(cam_id).at(i));
              temp->line_uvs_norm.at(cam_id).push_back(feat.second->line_uvs_norm.at(cam_id).at(i));
            }
          }
        }
      }
    }

    else {
      // Else we have not found the feature, so lets make it be a new one!
      std::shared_ptr<LineFeature> temp = std::make_shared<LineFeature>();
      temp->featid = feat.second->featid;
      temp->timestamps = feat.second->timestamps;
      temp->line_uvs = feat.second->line_uvs;
      temp->line_uvs_norm = feat.second->line_uvs_norm;
      features_idlookup[feat.first] = temp;
    }
  }
}

void LineFeatureDatabase::cleanup_measurements(double timestamp) {
  std::lock_guard<std::mutex> lck(line_mtx);
  for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    // Remove the older measurements
    (*it).second->clean_older_measurements(timestamp);
    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it).second->timestamps) {
      ct_meas += (int)(pair.second.size());
    }
    // If delete flag is set, then delete it
    if (ct_meas < 1) {
      features_idlookup.erase(it++);
    } else {
      it++;
    }
  }
}

