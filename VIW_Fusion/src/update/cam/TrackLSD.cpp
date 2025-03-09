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

#include "TrackLSD.h"
#include <ros/ros.h>
using namespace ov_core;
using namespace viw;
TrackLSD::TrackLSD (std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> cameras, bool stereo, ov_core::TrackBase::HistogramMethod histmethod, map<int, shared_ptr<ov_core::TrackBase>> _trackFEATS) 
        : camera_calib(cameras), use_stereo (stereo), histogram_method(histmethod), trackFEATS(_trackFEATS), database(new LineFeatureDatabase) {
  currid = 1;
  if (mtx_feeds.empty() || mtx_feeds.size() != camera_calib.size()) {
    std::vector<std::mutex> list(camera_calib.size());
    mtx_feeds.swap(list);
  }
}

void TrackLSD::feed_new_camera(const ov_core::CameraData &message, std::vector<Eigen::Vector2d> &vanishing_points) {
  // Error check that we have all the data
  if (message.sensor_ids.empty() || message.sensor_ids.size() != message.images.size() || message.images.size() != message.masks.size()) {
    PRINT_ERROR(RED "[ERROR]: MESSAGE DATA SIZES DO NOT MATCH OR EMPTY!!!\n" RESET);
    PRINT_ERROR(RED "[ERROR]:   - message.sensor_ids.size() = %zu\n" RESET, message.sensor_ids.size());
    PRINT_ERROR(RED "[ERROR]:   - message.images.size() = %zu\n" RESET, message.images.size());
    PRINT_ERROR(RED "[ERROR]:   - message.masks.size() = %zu\n" RESET, message.masks.size());
    std::exit(EXIT_FAILURE);
  }
 
  // Either call our stereo or monocular version
  // If we are doing binocular tracking, then we should parallize our tracking
  size_t num_images = message.images.size();
  if (num_images == 1) {
    feed_monocular(message, 0, vanishing_points);
  } else if (num_images == 2 && use_stereo) {
    // TODO:: add stereo line detetcion algorithem
    // feed_stereo(message, 0, 1);
    feed_monocular(message, 0, vanishing_points);
  } else if (!use_stereo) {
    parallel_for_(cv::Range(0, (int)num_images), LambdaBody([&](const cv::Range &range) {
                    for (int i = range.start; i < range.end; i++) {
                      feed_monocular(message, i, vanishing_points);
                    }
                  }));
  } else {
    PRINT_ERROR(RED "[ERROR]: invalid number of images passed %zu, we only support mono or stereo tracking", num_images);
    std::exit(EXIT_FAILURE);
  }
}

void TrackLSD::feed_monocular(const ov_core::CameraData &message, size_t msg_id, std::vector<Eigen::Vector2d> &vanishing_points) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Lock this data feed for this camera
  size_t cam_id = message.sensor_ids.at(msg_id);
  double timestamp = message.timestamp;
  std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));

  // Histogram equalize
  cv::Mat img, mask;
  if (histogram_method == ov_core::TrackBase::HistogramMethod::HISTOGRAM) {
    cv::equalizeHist(message.images.at(msg_id), img);
  } else if (histogram_method == ov_core::TrackBase::HistogramMethod::CLAHE) {
    double eq_clip_limit = 10.0;
    cv::Size eq_win_size = cv::Size(8, 8);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
    clahe->apply(message.images.at(msg_id), img);
  } else {
    img = message.images.at(msg_id);
  }
  mask = message.masks.at(msg_id);

  // If we are the first frame (or have lost tracking), initialize our descriptors
  if (lines_last.find(cam_id) == lines_last.end() || lines_last[cam_id].empty()) {
    std::vector<Eigen::Vector4f> good_left;
    std::vector<size_t> good_ids_left;
    perform_detection_monocular(img, mask, good_left, good_ids_left, vanishing_points);

    std::vector<cv::KeyPoint> points_new = trackFEATS.at(cam_id)->get_last_obs().at(cam_id);
    std::vector<size_t> point_ids_new = trackFEATS.at(cam_id)->get_last_ids().at(cam_id);
    std::vector<std::map<int, double>> point_on_lines_new;
    std::vector<Eigen::Vector4f> filted_line;
    std::vector<size_t> filted_id;
    std::vector<std::vector<Eigen::Vector2f>> point_position;
    AssignPointToLines(good_left, good_ids_left, points_new, point_ids_new, point_on_lines_new, point_position, filted_line, filted_id, timestamp);

    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id] = img;
    img_mask_last[cam_id] = mask;
    lines_last[cam_id] = filted_line;
    ids_last[cam_id] = filted_id;
    point_on_lines_last[cam_id] = point_on_lines_new;
    return;
  }

  // Our new keyline
  std::vector<Eigen::Vector4f> lines_new;
  std::vector<size_t> ids_new; 
  
  // extract lines for the new image
  perform_detection_monocular(img, mask, lines_new, ids_new, vanishing_points);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Lets match temporally using the match result of fature points
  // First assign current feature point to the lines
  std::vector<cv::KeyPoint> points_new = trackFEATS.at(cam_id)->get_last_obs().at(cam_id);
  // std::cout<<"There are feature points " <<  points_new.size() <<endl;
  std::vector<size_t> point_ids_new = trackFEATS.at(cam_id)->get_last_ids().at(cam_id);
  std::vector<std::map<int, double>> point_on_lines_new;
  std::vector<Eigen::Vector4f> filted_line;
  std::vector<size_t> filted_id;
  std::vector<std::vector<Eigen::Vector2f>> point_position;
  AssignPointToLines(lines_new, ids_new, points_new, point_ids_new, point_on_lines_new, point_position, filted_line, filted_id, timestamp);

  // Then try to match the line based on the point matching results
  std::map<int, int> line_matches;
  LineMatch(filted_line, lines_last[cam_id], point_on_lines_last[cam_id], point_on_lines_new, cam_id, cam_id, line_matches);
  rT3 = boost::posix_time::microsec_clock::local_time();
  
  // Get our "good tracks"
  std::vector<Eigen::Vector4f> good_lines;
  std::vector<size_t> good_ids;
  
  // Loop through all current left to right points
  for (size_t i = 0; i < filted_line.size(); i++) {
    // find the correct ID
    int id = -1;
    // Then lets replace the current ID with the old ID if found in last frame
    // Else just append the current feature and its unique ID
    if (!(line_matches.find(i) == line_matches.end())) {
      id = ids_last[cam_id][line_matches[i]];
    } else {
      id = filted_id[i];
    }
    good_lines.push_back(filted_line[i]);
    good_ids.push_back(id);
  }
  // cout << "The mathch number of line is " << match_number << endl;
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Update our line feature database, with theses new observations
  for (size_t i = 0; i < good_lines.size(); i++) {
    Eigen::Vector4f line_n = camera_calib.at(cam_id)->undistort_line(good_lines.at(i));
    int D = LineClassification(good_lines.at(i), vanishing_points);
    database->update_feature(good_ids.at(i), timestamp, cam_id, good_lines.at(i), line_n, point_on_lines_new.at(i), point_position.at(i), D);
  }
  
  // dispaly the detection and matching result for debug
  cv::Mat img_color = DrawFeatures(img, points_new, point_ids_new, good_lines, good_ids, point_on_lines_new, line_matches);
  imshow("Detection result", img_color);
  cv::waitKey(1);

  // Move forward in time
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id] = img;
    img_mask_last[cam_id] = mask;
    lines_last[cam_id] = good_lines;
    ids_last[cam_id] = good_ids;
    point_on_lines_last[cam_id] = point_on_lines_new;
  }
  rT5 = boost::posix_time::microsec_clock::local_time();
  
  // Our timing information
  PRINT_ALL("[TIME-DESC]: %.4f seconds for detection\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-DESC]: %.4f seconds for matching\n", (rT3 - rT2).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-DESC]: %.4f seconds for merging\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-DESC]: %.4f seconds for feature DB update (%d features)\n", (rT5 - rT4).total_microseconds() * 1e-6,
            (int)good_lines.size());
  PRINT_ALL("[TIME-DESC]: %.4f seconds for total\n", (rT5 - rT1).total_microseconds() * 1e-6);
}

void TrackLSD::perform_detection_monocular (const cv::Mat &img0, const cv::Mat &mask0, std::vector<Eigen::Vector4f> &lines0, 
                                      std::vector<size_t> &ids0, std::vector<Eigen::Vector2d> &vanishing_point) {

  // Extract our line features
  std::vector<cv::Vec4f> cv_lines;
  // creat a LSD line detector
  std::shared_ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold, distance_threshold, canny_th1, canny_th2, canny_aperture_size, false);
  
  // detect the line feature
  cv::Mat smaller_image;
  cv::resize(img0, smaller_image, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
  fld->detect(smaller_image, cv_lines);

  // std::Shared_Ptr<line_descriptor::LSDDetectorC> lsd_ = line_descriptor::LSDDetectorC::createLSDDetectorC();
  // Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
  // cv::Mat mask = Mat::ones( smaller_image.size(), CV_8UC1 );
  // std::vector<cv::line_descriptor::KeyLine> keylines;
  // lsd_->detect(smaller_image, keylines, mask);
  // cv::Mat lbd_descr;
  // bd->compute(smaller_image, keylines, lbd_descr);

  std::vector<Eigen::Vector4f> source_lines;
  // std::vector<cv::Vec4f> lines_o;
  // convert cv::vector to Eigen::Vector4f
  for(auto& cv_line : cv_lines){
    source_lines.emplace_back(cv_line[0]*2, cv_line[1]*2, cv_line[2]*2, cv_line[3]*2);
    // lines_o.emplace_back(cv_line[0]*2, cv_line[1]*2, cv_line[2]*2, cv_line[3]*2);
  }

  std::vector<Eigen::Vector4f> tmp_lines;
  // MergeLines(source_lines, tmp_lines, 0.05, 5, 15);
  // FilterShortLines(tmp_lines, 30);
  // MergeLines(tmp_lines, lines0, 0.03, 3, 30);
  // FilterShortLines(lines0, 60);
  
  lines0 = source_lines;
  // MergeLines(source_lines, lines0, 0.03, 2, 10);
  FilterShortLines(lines0, 40);
  // cout << "THe extraxtion line number is " << lines0.size() << endl;
  for (size_t i = 0; i < lines0.size(); i++) {
    size_t temp = ++currid;
    ids0.push_back(temp);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////// Show the line classification results !!! ////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Show found lines with FLD
  Mat img_color;
  cv::cvtColor(img0, img_color, cv::COLOR_GRAY2RGB);

  // Draw the vanishing point
  cv::Scalar color = cv::Scalar((int)0, (int)0, (int)255);
  cv::circle(img_color, cv::Point2i((int)vanishing_point[0].x(), (int)vanishing_point[0].y()), 10, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  cv::circle(img_color, cv::Point2i((int)vanishing_point[1].x(), (int)vanishing_point[1].y()), 10, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  cv::circle(img_color, cv::Point2i((int)vanishing_point[2].x(), (int)vanishing_point[2].y()), 10, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

  // Classification lines
  std::vector<std::vector<Eigen::Vector4f>> class_result(4);
  Classification(lines0, vanishing_point, class_result);
  for (auto& line : class_result[0]){
    cv::line(img_color, cv::Point2i((int)line(0), (int)line(1)), cv::Point2i((int)line(2), (int)line(3)), cv::Scalar(0, 0, 255), 2);
  }
  for (auto& line : class_result[1]){
    cv::line(img_color, cv::Point2i((int)line(0), (int)line(1)), cv::Point2i((int)line(2), (int)line(3)), cv::Scalar(0, 255, 0), 2);
  }
  for (auto& line : class_result[2]){
    cv::line(img_color, cv::Point2i((int)line(0), (int)line(1)), cv::Point2i((int)line(2), (int)line(3)), cv::Scalar(255, 0, 0), 2);
  }
  // for (auto& line : class_result[3]){
  //   cv::line(img_color, cv::Point2i((int)line(0), (int)line(1)), cv::Point2i((int)line(2), (int)line(3)), cv::Scalar(0, 255, 255), 2);
  // }
  imshow("Classification result", img_color);
  cv::waitKey(1);
  imshow("Raw Image", img0);
  cv::waitKey(1);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////// Show the line detection results !!! ////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Show found lines with FLD
  Mat img_color2;
  cv::cvtColor(img0, img_color2, cv::COLOR_GRAY2RGB);
  for (auto& line : lines0){
    cv::line(img_color2, cv::Point2i((int)line(0), (int)line(1)), cv::Point2i((int)line(2), (int)line(3)), cv::Scalar(0, 0, 255), 2);
  }
  imshow("Line Detection result", img_color2);
  cv::waitKey(1);

  // Displacy the extraction results
  // Mat line_image_fld(img0);
  // std::vector<cv::Vec4f> display_lines;
  // for (auto& dst_line : lines0) {
  //   display_lines.emplace_back(dst_line(0), dst_line(1), dst_line(2), dst_line(3));
  // }
  // fld->drawSegments(line_image_fld, display_lines);
  // imshow("FLD result", line_image_fld);
  // cv::waitKey(1);
}

void TrackLSD::Classification(const vector<Eigen::Vector4f> &lines, const vector<Eigen::Vector2d> &Vabishing_points, vector<vector<Eigen::Vector4f>> &class_result) {
  Eigen::Vector2d x_vp = Vabishing_points[0];
  Eigen::Vector2d y_vp = Vabishing_points[1];
  Eigen::Vector2d z_vp = Vabishing_points[2];
  for (auto &line : lines) {
    Eigen::Vector4f line_cp;
    line_cp << line(0), line(1), line(2), line(3);
    if (LineClass(line_cp, y_vp)) {
      class_result[1].push_back(line_cp);
      continue;
    } else if (LineClass(line_cp, x_vp)) {
      class_result[0].push_back(line_cp);
      continue;
    } else if (LineClass(line_cp, z_vp)) {
      class_result[2].push_back(line_cp);
      continue;
    } else {
      class_result[3].push_back(line_cp);
      continue;
    }
  }
}

int TrackLSD::LineClassification(const Eigen::Vector4f &line, const vector<Eigen::Vector2d> &Vabishing_points) {

  Eigen::Vector2d x_vp = Vabishing_points[0];
  Eigen::Vector2d y_vp = Vabishing_points[1];
  Eigen::Vector2d z_vp = Vabishing_points[2];

  if (LineClass(line, z_vp)) {
    return 3;
  } else if (LineClass(line, y_vp)) {
    return 2;
  } else if (LineClass(line, x_vp)) {
    return 1;
  } else {
    return 0;
  }
}

bool TrackLSD::LineClass(const Eigen::Vector4f &line, const Eigen::Vector2d &vanish_point) {

  // TODO:: Angle normalize
  Eigen::Vector3d s_point, e_point;
  s_point << line(0), line(1), 1;
  e_point << line(2), line(3), 1;
  Eigen::Vector3d mid_point = (s_point + e_point) / 2;
  
  // calculate the distance error
  Eigen::Vector3d line_new;
  Eigen::Vector3d vanish_point_3d(vanish_point(0), vanish_point(1), 1);
  line_new = mid_point.cross(vanish_point_3d);
  double dis_error = ((abs((line_new.transpose() * s_point).norm()) + abs((line_new.transpose() * e_point).norm())) / (2 * sqrt(line_new.x() * line_new.x() + line_new.y() * line_new.y())));
  dis_error = abs(dis_error);
  // calculate the angle error
  double angle1 = std::atan(line(1) - line(3)) / (line(0) - line(2));
  double angle2 = std::atan(mid_point(1) - vanish_point(1)) / (mid_point(0) - vanish_point(0));
  double angle_error = angle1 - angle2;
  // if (angle_error < -PI) {
  //   angle_error += 2 * PI;
  // } else if (angle_error > PI) {
  //   angle_error -= 2 * PI;
  // }
  angle_error = abs(angle_error);
  // angle_error = min(abs(angle_error), abs(PI - angle_error));

  if (dis_error <= 5.0 && angle_error <= 0.35) {
    return true;
  } else {
    return false;
  }
}

void TrackLSD::LineMatch(const std::vector<Eigen::Vector4f> lines_new, const std::vector<Eigen::Vector4f> lines_last, 
                        const std::vector<std::map<int, double>> &points_on_line0, const std::vector<std::map<int, double>> &points_on_line1, 
                          size_t id0, size_t id1, std::map<int, int> &line_matches) {
  rT3 = boost::posix_time::microsec_clock::local_time();
  size_t line_num0 = points_on_line0.size();
  size_t line_num1 = points_on_line1.size();
  int line_match_num = 0;

  if(line_num0 == 0 || line_num1 == 0) return;
  // fill in matching matrix
  Eigen::MatrixXi matching_matrix = Eigen::MatrixXi::Zero(line_num0, line_num1);
  for (size_t i = 0; i < line_num1; i++) {
    if (points_on_line1[i].size() < 1)
      continue;
    for (size_t j = 0; j < line_num0; j++) {
      if (points_on_line0[j].size() < 1)
        continue;
      
      for (auto &point : points_on_line0[j]) {

        if (points_on_line1[i].find(point.first) == points_on_line1[i].end()){
          continue;
        } else {
          matching_matrix(j, i) += 1;
          if (matching_matrix(j,i) >= 2){
            line_matches[i] = j;
            line_match_num++;
            break;
          } 
          else if (matching_matrix(j, i) == 1 && LineSimilar(lines_new[i],lines_last[j])){
            line_matches[i] = j;
            line_match_num++;
            break;
          }
          else
            continue;
        }
      }
    }
  }
  // std::cout << "The line matching number is " << line_matches.size() << endl;
  // std::cout << "The total number of lines is " << lines_last.size() << endl;

  // rT4 = boost::posix_time::microsec_clock::local_time();
  // find good matches
  // std::vector<int> row_max_value(line_num0), col_max_value(line_num1);
  // std::vector<Eigen::VectorXi::Index> row_max_location(line_num0), col_max_location(line_num1);
  // for (size_t i = 0; i < line_num0; i++) {
  //   row_max_location[i] = matching_matrix.row(i).maxCoeff(&row_max_location[i]);
  // }
  // for (size_t j = 0; j < line_num1; j++) {
  //   Eigen::VectorXi::Index col_max_location;
  //   int col_max_val = matching_matrix.col(j).maxCoeff(&col_max_location);
  //   // if(col_max_val < 2 || row_max_location[col_max_location] != j) continue;
  //   if(col_max_val < 1) continue;
  //   // float score = (float)(col_max_val * col_max_val) / std::min(points_on_line0[col_max_location].size(), points_on_line1[j].size());
  //   // if(score < 0.8) continue;

  //   // line_matches[col_max_location] = j;
  //   line_matches[j] = col_max_location;
  //   line_match_num++;
  // }
  // cout << "The line match number is " << line_match_num << std::endl;
  // rT4 = boost::posix_time::microsec_clock::local_time();
  // cout << "[TIME-DESC]:" << (rT4 - rT3).total_microseconds() * 1e-6 << std::endl;
}

void TrackLSD::FilterShortLines(std::vector<Eigen::Vector4f>& lines, float length_thr){
  Eigen::Array4Xf line_array = Eigen::Map<Eigen::Array4Xf, Eigen::Unaligned>(lines[0].data(), 4, lines.size());
  Eigen::ArrayXf length_square = (line_array.row(2) - line_array.row(0)).square() + (line_array.row(3) - line_array.row(1)).square();
  float thr_square = length_thr * length_thr;

  size_t long_line_num = 0;
  for(size_t i = 0; i < lines.size(); i++){
    if(length_square(i) > thr_square){
      lines[long_line_num] = lines[i];
      long_line_num++;
    }
  }
  lines.resize(long_line_num);
}

void TrackLSD::MergeLines(std::vector<Eigen::Vector4f> &source_lines, std::vector<Eigen::Vector4f> &dst_lines, float angle_threshold, 
    float distance_threshold, float endpoint_threshold) {
  size_t source_line_num = source_lines.size();
  Eigen::Array4Xf line_array = Eigen::Map<Eigen::Array4Xf, Eigen::Unaligned>(source_lines[0].data(), 4, source_lines.size());
  Eigen::ArrayXf x1 = line_array.row(0);
  Eigen::ArrayXf y1 = line_array.row(1);
  Eigen::ArrayXf x2 = line_array.row(2);
  Eigen::ArrayXf y2 = line_array.row(3);

  Eigen::ArrayXf dx = x2 - x1;
  Eigen::ArrayXf dy = y2 - y1;
  Eigen::ArrayXf eigen_angles = (dy / dx).atan();
  Eigen::ArrayXf length = (dx * dx + dy * dy).sqrt();

  std::vector<float> angles(&eigen_angles[0], eigen_angles.data()+eigen_angles.cols()*eigen_angles.rows());
  std::vector<size_t> indices(angles.size());                                                        
  std::iota(indices.begin(), indices.end(), 0);                                                      
  std::sort(indices.begin(), indices.end(), [&angles](size_t i1, size_t i2) { return angles[i1] < angles[i2]; });

  // search clusters
  float angle_thr = angle_threshold;
  float distance_thr = distance_threshold * 5;
  float ep_thr = endpoint_threshold * endpoint_threshold;
  float quater_PI = M_PI / 4.0;

  std::vector<std::vector<size_t>> neighbors;
  neighbors.resize(source_line_num);
  std::vector<bool> sort_by_x;
  for(size_t i = 0; i < source_line_num; i++){
    size_t idx1 = indices[i];
    float x11 = source_lines[idx1](0);
    float y11 = source_lines[idx1](1);
    float x12 = source_lines[idx1](2);
    float y12 = source_lines[idx1](3);
    float angle1 = angles[idx1];
    bool to_sort_x = (std::abs(angle1) < quater_PI);
    sort_by_x.push_back(to_sort_x);
    if((to_sort_x && (x12 < x11)) || ((!to_sort_x) && y12 < y11)){
      std::swap(x11, x12);
      std::swap(y11, y12);
    }

    for(size_t j = i +1; j < source_line_num; j++){
      size_t idx2 = indices[j];
      float x21 = source_lines[idx2](0);
      float y21 = source_lines[idx2](1);
      float x22 = source_lines[idx2](2);
      float y22 = source_lines[idx2](3);
      if((to_sort_x && (x22 < x21)) || ((!to_sort_x) && y22 < y21)){
        std::swap(x21, x22);
        std::swap(y21, y22);
      }

      // check delta angle
      float angle2 = angles[idx2];
      float d_angle = AngleDiff(angle1, angle2);
      if(d_angle > angle_thr){
        if(std::abs(angle1) < (M_PI_2 - angle_threshold)){
          break;
        }else{
          continue;
        }
      }

      // check distance
      Eigen::Vector2f mid1 = 0.5 * (source_lines[idx1].head(2) + source_lines[idx1].tail(2));
      Eigen::Vector2f mid2 = 0.5 * (source_lines[idx2].head(2) + source_lines[idx2].tail(2));
      float mid1_to_line2 = PointLineDistance(source_lines[idx2], mid1);
      float mid2_to_line1 = PointLineDistance(source_lines[idx1], mid2);
      if(mid1_to_line2 > distance_thr && mid2_to_line1 > distance_thr) continue;

      // check endpoints distance
      float cx12, cy12, cx21, cy21;
      if((to_sort_x && x12 > x22) || (!to_sort_x && y12 > y22)){
        cx12 = x22;
        cy12 = y22;
        cx21 = x11;
        cy21 = y11;
      }else{
        cx12 = x12;
        cy12 = y12;
        cx21 = x21;
        cy21 = y21;
      }
      bool to_merge = ((to_sort_x && cx12 >= cx21) || (!to_sort_x && cy12 >= cy21));
      if(!to_merge){
        float d_ep = (cx21 - cx12) * (cx21 - cx12) + (cy21 - cy12) * (cy21 - cy12);
        to_merge = (d_ep < ep_thr);
      }

      // check cluster code
      if(to_merge){
        neighbors[idx1].push_back(idx2);
        neighbors[idx2].push_back(idx1);
      }
    }
  }

  // clusters
  std::vector<int> cluster_codes(source_line_num, -1);
  std::vector<std::vector<size_t>> cluster_ids;
  for(size_t i = 0; i < source_line_num; i++){
    if(cluster_codes[i] >= 0) continue;

    size_t new_code = cluster_ids.size();
    cluster_codes[i] = new_code;
    std::vector<size_t> to_check_ids = neighbors[i];
    std::vector<size_t> cluster;
    cluster.push_back(i);
    while(to_check_ids.size() > 0){
      std::set<size_t> tmp;
      for(auto& j : to_check_ids){
        if(cluster_codes[j] < 0){
          cluster_codes[j] = new_code;
          cluster.push_back(j);
        }

        std::vector<size_t> j_neighbor = neighbors[j];
        for(auto& k : j_neighbor){
          if(cluster_codes[k] < 0){
            tmp.insert(k);
          } 
        }
      }
      to_check_ids.clear();
      to_check_ids.assign(tmp.begin(), tmp.end());    
    }
    cluster_ids.push_back(cluster);
  }

  // search sub-cluster
  std::vector<std::vector<size_t>> new_cluster_ids;
  for(auto& cluster : cluster_ids){
    size_t cluster_size = cluster.size();
    if(cluster_size <= 2){
      new_cluster_ids.push_back(cluster);
      continue;
    }

    std::sort(cluster.begin(), cluster.end(), [&length](size_t i1, size_t i2) { return length(i1) > length(i2); });
    std::unordered_map<size_t, size_t> line_location;
    for(size_t i = 0; i < cluster_size; i++){
      line_location[cluster[i]] = i;
    }

    std::vector<bool> clustered(cluster_size, false);
    for(size_t j = 0; j < cluster_size; j++){
      if(clustered[j]) continue;

      size_t line_idx = cluster[j];
      std::vector<size_t> sub_cluster;
      sub_cluster.push_back(line_idx);
      std::vector<size_t> line_neighbors = neighbors[line_idx];
      for(size_t k : line_neighbors){
        clustered[line_location[k]] = true;
        sub_cluster.push_back(k);
      }
      new_cluster_ids.push_back(sub_cluster);
    }
  }

  // merge clusters
  dst_lines.clear();
  dst_lines.reserve(new_cluster_ids.size());
  for(auto& cluster : new_cluster_ids){
    size_t idx0 = cluster[0];
    Eigen::Vector4f new_line = source_lines[idx0];
    for(size_t i = 1; i < cluster.size(); i++){
      new_line = MergeTwoLines(new_line, source_lines[cluster[i]]);
    }
    dst_lines.push_back(new_line);
  }        
}

float TrackLSD::AngleDiff (float &angle1, float &angle2) {
  float d_angle_case1 = std::abs(angle2 - angle1);
  float d_angle_case2 = PI + std::min(angle1, angle2) - std::max(angle1, angle2);
  return std::min(d_angle_case1, d_angle_case2); 
}

bool TrackLSD::OverlapCheck (const Eigen::Vector4f &line1, const Eigen::Vector4f &line2) {

  float x11 = line1(0);
  float y11 = line1(1);
  float x12 = line1(2);
  float y12 = line1(3);
  float x21 = line2(0);
  float y21 = line2(1);
  float x22 = line2(2);
  float y22 = line2(3);
  // first check the projections of the two line segments on the x and y coordinates overlap
  if ((x11 > x12 ? x11 : x12) < (x21 < x22 ? x21 : x22) ||
      (y11 > y12 ? y11 : y12) < (y21 < y22 ? y21 : y22) ||
      (x21 > x22 ? x21 : x22) < (x11 < x12 ? x11 : x12) ||
      (y21 > y22 ? y21 : y22) < (y11 < y12 ? y11 : y12)) {
    return false;
  }

  // Vector cross product (to check two lines is overlap or not)
  if ((((x11 - x21) * (y22 - y21) - (y11 - y21) * (x22 - x21))*
       ((x12 - x21) * (y22 - y21) - (y12 - y21) * (x22 - x21))) > 0 ||
      (((x21 - x11) * (y12 - y11) - (y21 - y11) * (x12 - x11))*
       ((x22 - x11) * (y12 - y11) - (y22 - y11) * (x12 - x11))) > 0) {
    return false;
  }
  return true;
}

float TrackLSD::EndPointDistance (const Eigen::Vector4f &line1, const Eigen::Vector4f &line2) {

  float x11 = line1(0);
  float y11 = line1(1);
  float x12 = line1(2);
  float y12 = line1(3);
  float x21 = line2(0);
  float y21 = line2(1);
  float x22 = line2(2);
  float y22 = line2(3);

  float dl11_l22 = sqrt((x11 - x22) * (x11 - x22) + (y11 - y22) * (y11 - y22));
  float dl12_l21 = sqrt((x12 - x21) * (x12 - x21) + (y12 - y21) * (y12 - y21));
  float dl11_l21 = sqrt((x11 - x21) * (x11 - x21) + (y11 - y21) * (y11 - y21));
  float dl12_l22 = sqrt((x12 - x22) * (x12 - x22) + (y12 - y22) * (y12 - y22));

  float temp1 = std::min (dl11_l22, dl12_l21);
  float temp2 = std::min (dl11_l21, dl12_l22);

  return std::min (temp1 , temp2);
}

Eigen::Vector4f TrackLSD::MergeTwoLines (const Eigen::Vector4f &line1, const Eigen::Vector4f &line2) {
  // line 1 parameter
  float x11 = line1(0);
  float y11 = line1(1);
  float x12 = line1(2);
  float y12 = line1(3);
  
  float dx1 = (x12 - x11);
  float dy1 = (y12 - y11);
  float l1 = sqrt((double) (dx1 * dx1) + (double) (dy1 * dy1));

  // line 2 parameter
  float x21 = line2(0);
  float y21 = line2(1);
  float x22 = line2(2);
  float y22 = line2(3);

  float dx2 = (x22 - x21);
  float dy2 = (y22 - y21);
  float l2 = sqrt((double) (dx2 * dx2) + (double) (dy2 * dy2)); 

  // calculate the coffecient
  double xg = (l1 * (double) (x11 + x12) + l2 * (double) (x21 + x22))
      / (double) (2.0 * (l1 + l2));
  double yg = (l1 * (double) (y11 + y12) + l2 * (double) (y21 + y22))
      / (double) (2.0 * (l1 + l2));
  
  // the line theta 
  double th1, th2 = 0.0;
  if (dx1 == 0.0f) 
    th1 = PI;
  else th1 = atan(dy1 / dx1);

  if (dx2 == 0.0f) 
    th2 = PI;
  else th2 = atan(dy2 / dx2);

  double thr;
  if (fabs(th1 - th2) <= PI / 2.0) {
    thr = (l1 * th1 + l2 * th2) / (l1 + l2);
  }
  else {
    double temp = th2 - PI * (th2 / fabs(th2));
    thr = (l1 * th1 + l2 * temp) / (l1 + l2);
  }
  
  double axg = ((double) y11 - yg) * sin(thr) + ((double) x11 - xg) * cos(thr);
  double bxg = ((double) y12 - yg) * sin(thr) + ((double) x12 - xg) * cos(thr);
  double cxg = ((double) y21 - yg) * sin(thr) + ((double) x21 - xg) * cos(thr);
  double dxg = ((double) y22 - yg) * sin(thr) + ((double) x22 - xg) * cos(thr);

  double delta1xg = std::min(axg, std::min(bxg, std::min(cxg,dxg)));
  double delta2xg = std::max(axg, std::max(bxg, std::max(cxg,dxg)));
  
  double delta1x = delta1xg * std::cos(thr) + xg;
  double delta1y = delta1xg * std::sin(thr) + yg;
  double delta2x = delta2xg * std::cos(thr) + xg;
  double delta2y = delta2xg * std::sin(thr) + yg;

  Eigen::Vector4f new_line;
  new_line << (float)delta1x, (float)delta1y, (float)delta2x, (float)delta2y;
  return new_line;  
}

void TrackLSD::AssignPointToLines (std::vector<Eigen::Vector4f> &lines, std::vector<size_t> &lines_id, std::vector<cv::KeyPoint> &points, std::vector<size_t> ids,
                                    std::vector<std::map<int, double>> &relation, std::vector<std::vector<Eigen::Vector2f>> &point_position, std::vector<Eigen::Vector4f> &new_lines, 
                                    std::vector<size_t> &new_id, double timestamp){
  relation.clear();
  relation.reserve(lines.size());

  point_position.clear();
  point_position.reserve(lines.size());

  for (size_t i = 0; i < lines.size(); i++) {
    double lx1 = lines[i](0);
    double lx2 = lines[i](1);
    double ly1 = lines[i](2);
    double ly2 = lines[i](3);

    // find the min and max x,y
    double min_lx = lx1;
    double max_lx = lx2;
    double min_ly = ly1;
    double max_ly = ly2;
    if(lx1 > lx2) std::swap(min_lx, max_lx);
    if(ly1 > ly2) std::swap(min_ly, max_ly);

    std::map<int, double> points_on_line;
    std::vector<Eigen::Vector2f> features_points;
    bool find_point = false;
    for (size_t j = 0; j < points.size(); j++) {
      float x = points[j].pt.x;
      float y = points[j].pt.y;
      Eigen::Vector2f point(x, y);
      // check if coordinate is bigger than the max end-point value
      if (point(0) < min_lx || point(0) > max_lx || point(1) < min_ly || point(1) > max_ly){
        continue;
      }
      // check the distance btween point and line
      float distance = PointLineDistance(lines[i], point);
      if (distance > 5) continue;
      points_on_line[ids[j]] = distance;
      features_points.push_back(point);
      find_point = true;
    }
    if (find_point){
      relation.push_back(points_on_line);
      new_lines.push_back(lines[i]);
      new_id.push_back(lines_id[i]);
      point_position.push_back(features_points);
    }
  }
}

float TrackLSD::PointLineDistance(const Eigen::Vector4f &line, const Eigen::Vector2f &point) {
  float x0 = point(0);
  float y0 = point(1);
  float x1 = line(0);
  float y1 = line(1);
  float x2 = line(2);
  float y2 = line(3);
  // float d = std::abs(std::fabs((y2 - y1) * x0 +(x1 - x2) * y0 + ((x2 * y1) -(x1 * y2))) / (std::sqrt(std::pow(y2 - y1, 2) + std::pow(x1 - x2, 2))));
  // return d;

  float cross = (x2 - x1) * (x0 - x1) + (y2 - y1) * (y0 - y1);
  if (cross <= 0) {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  } 
  float d = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  if (cross > d) {
    return std::sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
  }

  return std::abs(std::fabs((y2 - y1) * x0 +(x1 - x2) * y0 + ((x2 * y1) -(x1 * y2))) / (std::sqrt(std::pow(y2 - y1, 2) + std::pow(x1 - x2, 2))));
}

bool TrackLSD::LineSimilar (const Eigen::Vector4f &line2, const Eigen::Vector4f &line1){

  float x3 = line1(0);
  float y3 = line1(1);
  float x4 = line1(2);
  float y4 = line1(3);
  Eigen::Vector2f mid_point;
  mid_point << (x3 + x4) / 2, (y3 + y4) /2;
  if (PointLineDistance(line2, mid_point) <= 6) {
    return true;
  }
  else {
    return false;
  }
}

cv::Mat TrackLSD::DrawFeatures(const cv::Mat &img, const std::vector<cv::KeyPoint> &points, const std::vector<size_t> &point_ids, const std::vector<Eigen::Vector4f> &lines,
                      const std::vector<size_t> &line_ids, const std::vector<std::map<int, double>> &point_on_lines, std::map<int, int> line_matches) {
  // convert the grey image to color image 
  cv::Mat img_color;
  cv::cvtColor(img, img_color, cv::COLOR_GRAY2RGB);
  // size_t point_num = points.size();
  std::unordered_map<int, cv::Scalar> colors;
  std::unordered_map<int, int> radii;
  std::unordered_map<int, cv::KeyPoint> feature_map;
  for (size_t i = 0; i < points.size(); i++){
    feature_map[point_ids[i]] = points[i];
  }

  for (size_t i = 0; i < lines.size(); i++){
    if (line_ids[i] < 0) {
      continue;
    }
    // if (line_matches.find(i) == line_matches.end()) continue;
    LineFeature line;
    if (!database->get_feature_clone(line_ids[i], line)){
        continue;
    }
    // for (size_t j = 0; j < line.line_uvs[0].size() - 1; j++){
    //   int color_b = 255 + (int)(1.0 * 255 / line.line_uvs[0].size() * j);
    //   int color_g = 0 + (int)(1.0 * 255 / line.line_uvs[0].size() * j);
    //   int color_r = 0 + (int)(1.0 * 255 / line.line_uvs[0].size() * j);
    //   Eigen::Vector4f line_c = line.line_uvs[0][j];
    //   Eigen::Vector4f line_h = line.line_uvs[0][j+1];
    //   cv::line(img_color, cv::Point2i((int)((line_c(0)+line_c(2))/2), (int)((line_c(1) + line_c(3))/2)), 
    //     cv::Point2i((int)((line_h(0)+line_h(2))/2), (int)((line_h(1) + line_h(3))/2)), cv::Scalar(color_r, color_g, color_b), 2);
    // }
    cv::Scalar color = cv::Scalar((int)(i * 128)%255, (int)(i * 53)%255, (int)(i * 23)%255);
    Eigen::Vector4f linev = lines[i];
    cv::line(img_color, cv::Point2i((int)(linev(0)+0.5), (int)(linev(1)+0.5)), 
        cv::Point2i((int)(linev(2)+0.5), (int)(linev(3)+0.5)), color, 2);
    // cv::putText(img_color, std::to_string(line_ids[i]), cv::Point((int)((linev(0)+linev(2))/2), 
    //     (int)((linev(1)+linev(3))/2)), cv::FONT_HERSHEY_DUPLEX, 1.0, color, 2);
    for (auto &kv : point_on_lines[i]) {
      colors[kv.first] = color;
      radii[kv.first] *= 2;
      cv::KeyPoint point = feature_map[kv.first];
      cv::circle(img_color, point.pt, 8, colors[kv.first], 2, cv::LINE_AA);
    }
  }
  
  // // draw points
  // for(auto &point : feature_map){
  //   if (!(colors.find(point.first) == colors.end())){
  //     cv::circle(img_color, point.second.pt, 8, colors[point.first], 2, cv::LINE_AA);
  //   } 
  //   // else
  //   //   cv::circle(img_color, point.second.pt, 4, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
  // }

  return img_color;
}