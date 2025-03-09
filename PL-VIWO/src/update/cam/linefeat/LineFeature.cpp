#include "LineFeature.h"

using namespace viw;

void LineFeature::clean_old_measurements(const std::vector<double> &valid_times) {

  // Loop through each of the cameras we have
  for (auto const &pair : timestamps) {

    // Assert that we have all the parts of a measurement
    assert(timestamps[pair.first].size() == line_uvs[pair.first].size());
    assert(timestamps[pair.first].size() == line_uvs_norm[pair.first].size());

    // Our iterators
    auto it1 = timestamps[pair.first].begin();
    auto it2 = line_uvs[pair.first].begin();
    auto it3 = line_uvs_norm[pair.first].begin();

    // Loop through measurement times, remove ones that are not in our timestamps
    while (it1 != timestamps[pair.first].end()) {
      if (std::find(valid_times.begin(), valid_times.end(), *it1) == valid_times.end()) {
        it1 = timestamps[pair.first].erase(it1);
        it2 = line_uvs[pair.first].erase(it2);
        it3 = line_uvs_norm[pair.first].erase(it3);
      } else {
        ++it1;
        ++it2;
        ++it3;
      }
    }
  }
}

void LineFeature::clean_invalid_measurements(const std::vector<double> &invalid_times) {

  // Loop through each of the cameras we have
  for (auto const &pair : timestamps) {

    // Assert that we have all the parts of a measurement
    assert(timestamps[pair.first].size() == line_uvs[pair.first].size());
    assert(timestamps[pair.first].size() == line_uvs_norm[pair.first].size());

    // Our iterators
    auto it1 = timestamps[pair.first].begin();
    auto it2 = line_uvs[pair.first].begin();
    auto it3 = line_uvs_norm[pair.first].begin();

    // Loop through measurement times, remove ones that are in our timestamps
    while (it1 != timestamps[pair.first].end()) {
      if (std::find(invalid_times.begin(), invalid_times.end(), *it1) != invalid_times.end()) {
        it1 = timestamps[pair.first].erase(it1);
        it2 = line_uvs[pair.first].erase(it2);
        it3 = line_uvs_norm[pair.first].erase(it3);
      } else {
        ++it1;
        ++it2;
        ++it3;
      }
    }
  }
}

void LineFeature::clean_older_measurements(double timestamp) {

  // Loop through each of the cameras we have
  for (auto const &pair : timestamps) {

    // Assert that we have all the parts of a measurement
    assert(timestamps[pair.first].size() == line_uvs[pair.first].size());
    assert(timestamps[pair.first].size() == line_uvs_norm[pair.first].size());

    // Our iterators
    auto it1 = timestamps[pair.first].begin();
    auto it2 = line_uvs[pair.first].begin();
    auto it3 = line_uvs_norm[pair.first].begin();

    // Loop through measurement times, remove ones that are older then the specified one
    while (it1 != timestamps[pair.first].end()) {
      if (*it1 <= timestamp) {
        it1 = timestamps[pair.first].erase(it1);
        it2 = line_uvs[pair.first].erase(it2);
        it3 = line_uvs_norm[pair.first].erase(it3);
      } else {
        ++it1;
        ++it2;
        ++it3;
      }
    }
  }
}