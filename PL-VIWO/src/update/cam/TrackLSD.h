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

#ifndef VIW_TRACK_LSD_H
#define VIW_TRACK_LSD_H

#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>
#include <numeric>
#include <unordered_map>

#include "utils/colors.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include "cam/CamBase.h"
#include "track/TrackBase.h"
#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
//#include <opencv2/line_descriptor.hpp>

#include "utils/opencv_lambda_body.h"
#include "linefeat/LineFeatureDatabase.h"
#include "linefeat/LineFeature.h"

#define PI 3.141592654

using namespace std;
using namespace cv;

namespace ov_core {
  class TrackBase;
  class CameraData;
  class CamBase;
};

namespace viw {
  /**
   * @brief LSD line tracking of feature
   * 
   * This is a implementation of LSD line visual front-end for tracking line feature
   * We can track either monocular cameras across time (temporally) along with
   * stereo cameras which we also track across time (temporally) but track from left to right
   * to find the stereo correspondence information also.
   * This uses the 
   */
class TrackLSD {

public:
  /**
   * @brief Public constructor with configuration variables
   * @param cameras camera calibration object which has all camera intrinsics in it
   * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
   * @param stereo if we should do stereo feature tracking or binocular
   * @param histmethod what type of histogram pre-processing should be done (histogram eq?)
   */
  TrackLSD (std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> cameras, bool stereo, 
              ov_core::TrackBase::HistogramMethod histmethod, map<int, shared_ptr<ov_core::TrackBase>> _trackFEATS);

  virtual ~TrackLSD() {}

  /**
   * @brief Process a new image
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_new_camera(const ov_core::CameraData &message, std::vector<Eigen::Vector2d> &vanishing_points);

    /**
   * @brief Get the feature database with all the track information
   * @return FeatureDatabase pointer that one can query for features
   */
  std::shared_ptr<LineFeatureDatabase> get_feature_database() { return database; }

protected:
  /**
   * @brief Process a new monocular image
   * @param message Contains our timestamp, images, and camera ids
   * @param msg_id the camera index in message data vector
   */
  void feed_monocular(const ov_core::CameraData &message, size_t msg_id, std::vector<Eigen::Vector2d> &vanishing_points);

  /**
   * @brief Process new stereo pair of images
   * @param message Contains our timestamp, images, and camera ids
   * @param msg_id_left first image index in message data vector
   * @param msg_id_right second image index in message data vector
   */
  void feed_stereo(const ov_core::CameraData &message, size_t msg_id_left, size_t msg_id_right);

  /**
   * @brief Detects new features in the current image
   * @param img0 image we will detect features on
   * @param mask0 mask which has what ROI we do not want features in
   * @param lines0 vector of extracted feature lines
   * @param ids0 vector of all new IDs
   * Given a set of images, and their currently extracted features, this will try to add new features.
   * We return all extracted descriptors here since we DO NOT need to do stereo tracking left to right.
   * Our vector of IDs will be later overwritten when we match features temporally to the previous frame's features.
   * See robust_match() for the matching.
   */
  void perform_detection_monocular(const cv::Mat &img0, const cv::Mat &mask0, std::vector<Eigen::Vector4f> &lines0,
                                   std::vector<size_t> &ids0, std::vector<Eigen::Vector2d> &vanishing_points);
  /**
   * @brief Line matching using the point matching result
   * @param lines0 Vector of line0
   * @param lines1 Vector of line1
   * @param id0 Vector of line0 IDs
   * @param id1 Vector of line1 IDs
   * @param mask_out matching results
   */
  void LineMatch(const std::vector<Eigen::Vector4f> lines_new, const std::vector<Eigen::Vector4f> lines_last, 
                        const std::vector<std::map<int, double>> &points_on_line0, const std::vector<std::map<int, double>> &points_on_line1, 
                          size_t id0, size_t id1, std::map<int, int> &line_matches);
  /**
   * @brief Assign feature point to lines
   * @param lines Vector of lines 
   * @param points Vector of points
   * @param relation relation map between points and lines
   */
  void AssignPointToLines (std::vector<Eigen::Vector4f> &lines, std::vector<size_t> &lines_id, std::vector<cv::KeyPoint> &points, std::vector<size_t> ids,
                            std::vector<std::map<int, double>> &relation, std::vector<std::vector<Eigen::Vector2f>> &point_position, std::vector<Eigen::Vector4f> &new_lines, 
                            std::vector<size_t> &new_id, double timestamp);

  /**
   * @brief Filter the lines that smaller than the threshold
   * @param lines Vector of lines that need to be filted
   * @param length Length threshold
   */
  void FilterShortLines(std::vector<Eigen::Vector4f>& lines, float length_thr);

  /**
   * @brief Merge the lines that belong to same cluster
   * @param source_lines Vector of lines that need to be filted
   * @param dst_lines Length threshold
   * @param angle_threshold
   * @param distance_threshold
   * @param endpoint_threshold
   */
  void MergeLines(std::vector<Eigen::Vector4f> &source_lines, std::vector<Eigen::Vector4f> &dst_lines, float angle_threshold, 
    float distance_threshold, float endpoint_threshold);
  
  void Classification(const vector<Eigen::Vector4f> &lines, const vector<Eigen::Vector2d> &Vabishing_points, vector<vector<Eigen::Vector4f>> &class_result);

  bool LineClass(const Eigen::Vector4f &line, const Eigen::Vector2d &vanish_point); 

  int LineClassification(const Eigen::Vector4f &line, const vector<Eigen::Vector2d> &Vabishing_points);

  /**
   * @brief Calculate the angle difference
   * @param angle1 input angle 1
   * @param angle2 input angle 2
   */
  float AngleDiff (float &angle1, float &angle2);

  /**
   * @brief Calculate the distance from point to line
   * @param line input line
   * @param point input point
   */
  float PointLineDistance(const Eigen::Vector4f &line, const Eigen::Vector2f &point);
  
  /**
   * @brief check two lines is overlaped or not
   * @param line1
   * @param line2
   * https://blog.csdn.net/HelloZEX/article/details/80880385
   */
  bool OverlapCheck (const Eigen::Vector4f &line1, const Eigen::Vector4f &line2);
  
  /**
   * @brief find the shortest end-point distance of two lines
   * @param line1
   * @param line2
   */
  float EndPointDistance (const Eigen::Vector4f &line1, const Eigen::Vector4f &line2);

  /**
   * @brief merge two close lines
   * @param line1
   * @param line2
   */
  Eigen::Vector4f MergeTwoLines (const Eigen::Vector4f &line1, const Eigen::Vector4f &line2);

  /**
   * @brief Get the similarity between two 2D lines 
   * 
   */
  bool LineSimilar (const Eigen::Vector4f &line2, const Eigen::Vector4f &line1);
  /**
   * @brief Draw all point and line feature on the image
   * @param img raw image
   * @param points vector of point features
   * @param lines vector of lines features
   * @param line_ids vector of line ID
   * @param point_on_lines 
   */
  cv::Mat DrawFeatures(const cv::Mat &img, const std::vector<cv::KeyPoint> &points, const std::vector<size_t> &point_ids, const std::vector<Eigen::Vector4f> &lines,
                        const std::vector<size_t> &line_ids, const std::vector<std::map<int, double>> &point_on_lines, std::map<int, int> line_matches);

  /// Camera object which has all calibration in it
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> camera_calib;
  
  /// Database with all our current features
  std::shared_ptr<LineFeatureDatabase> database;
  
  /// If we should use binocular tracking or stereo tracking for multi-camera
  bool use_stereo;

  /// What histogram equalization method we should pre-process images with?
  ov_core::TrackBase::HistogramMethod histogram_method;

  /// Feature point tracker
  map<int, shared_ptr<ov_core::TrackBase>> trackFEATS;

  /// Mutexs for our last set of image storage (img_last, pts_last, and ids_last)
  std::vector<std::mutex> mtx_feeds;

  /// Mutex for editing the *_last variables
  std::mutex mtx_last_vars;

  /// Last set of images (use map so all trackers render in the same order)
  std::map<size_t, cv::Mat> img_last;

  /// Last set of images (use map so all trackers render in the same order)
  std::map<size_t, cv::Mat> img_mask_last;
  
    /// Set of IDs of each current feature in the database
  std::unordered_map<size_t, std::vector<size_t>> ids_last;

  /// Last set of tracked points
  std::unordered_map<size_t, std::vector<Eigen::Vector4f>> lines_last;
  
  /// Line assign with point in the last frame
  std::unordered_map<size_t, std::vector<std::map<int, double>>> point_on_lines_last;

  // Last set of image pyramids
  std::map<size_t, std::vector<cv::Mat>> img_pyramid_last;
  std::map<size_t, cv::Mat> img_curr;
  std::map<size_t, std::vector<cv::Mat>> img_pyramid_curr;

  // line detection cofig
  int length_threshold = 20;
  float distance_threshold = 1.414213562;
  double canny_th1 = 50.0;
  double canny_th2 = 50.0;
  int canny_aperture_size = 3;

  // Stereo feature map (feature id, is stereo)
  std::map<int, bool> stereo_map;

  /// Master ID for this tracker (atomic to allow for multi-threading)
  std::atomic<size_t> currid;

  // Timing variables (most children use these...)
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;
};

} // namespace viw
#endif /* VIW_TRACK_LSD_H*/