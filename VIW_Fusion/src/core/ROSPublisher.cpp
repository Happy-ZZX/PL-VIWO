#include "ROSPublisher.h"
#include "ROSHelper.h"
#include "SystemManager.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsWheel.h"
#include "sensor_msgs/PointCloud2.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "update/cam/UpdaterCamera.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/MathGPS.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "update/gps/UpdaterGPS.h"
#include "utils/Print_Logger.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;
using namespace viw;
using namespace ov_type;

ROSPublisher::ROSPublisher(shared_ptr<ros::NodeHandle> nh, shared_ptr<SystemManager> sys, shared_ptr<Options> op) : nh(nh), sys(sys), op(op) {

  // Setup our transform broadcaster
  mTfBr = make_shared<tf::TransformBroadcaster>();

  // Basic IMU publish
  pub_imu_pose = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/mins/imu/pose", 2);
  PRINT1("Publishing: %s\n", pub_imu_pose.getTopic().c_str());
  pub_imu_odom = nh->advertise<nav_msgs::Odometry>("/mins/imu/odom", 2);
  PRINT1("Publishing: %s\n", pub_imu_odom.getTopic().c_str());
  pub_imu_path = nh->advertise<nav_msgs::Path>("/mins/imu/path", 2);
  PRINT1("Publishing: %s\n", pub_imu_path.getTopic().c_str());

  // CAM
  if (sys->state->op->cam->enabled) {
    pub_cam_msckf = nh->advertise<sensor_msgs::PointCloud2>("/mins/cam/msckf", 2);
    PRINT1("Publishing: %s\n", pub_cam_msckf.getTopic().c_str());
    pub_cam_slam = nh->advertise<sensor_msgs::PointCloud2>("/mins/cam/slam", 2);
    PRINT1("Publishing: %s\n", pub_cam_slam.getTopic().c_str());
    pub_cam_line = nh->advertise<visualization_msgs::Marker>("/mins/cam/line", 2);
    PRINT1("Publishing: %s\n", pub_cam_line.getTopic().c_str());

    for (int i = 0; i < op->est->cam->max_n; i++) {
      image_transport::ImageTransport it(*nh); // Our tracking image
      pub_cam_image.push_back(it.advertise("/mins/cam" + to_string(i) + "/track_img", 2));
      PRINT1("Publishing: %s\n", pub_cam_image.back().getTopic().c_str());
    }
  }

  // GPS
  if (sys->state->op->gps->enabled) {
    for (int i = 0; i < sys->state->op->gps->max_n; i++) {
      seq_gps.push_back(0);
      pub_gps_pose.emplace_back();
      pub_gps_pose.back() = nh->advertise<geometry_msgs::PoseStamped>("/mins/gps" + to_string(i) + "/pose", 2);
      PRINT1("Publishing: %s\n", pub_gps_pose.back().getTopic().c_str());
      pub_gps_path.emplace_back();
      pub_gps_path.back() = nh->advertise<nav_msgs::Path>("/mins/gps" + to_string(i) + "/path", 2);
      PRINT1("Publishing: %s\n", pub_gps_path.back().getTopic().c_str());
    }
    path_gps = vector<vector<geometry_msgs::PoseStamped>>(sys->state->op->gps->max_n);
  }
}

void ROSPublisher::publish_imu() {

  // Return if we have not initialized
  if (!sys->state->initialized)
    return;

  // Our odometry message
  Matrix<double, 13, 1> odom;
  odom.block(0, 0, 4, 1) = sys->state->imu->quat();
  odom.block(4, 0, 3, 1) = sys->state->imu->pos();
  odom.block(7, 0, 3, 1) = sys->state->imu->vel();
  odom.block(10, 0, 3, 1) = sys->state->have_cpi(sys->state->time) ? sys->state->cpis.at(sys->state->time).w : Vector3d::Zero();
  nav_msgs::Odometry odomIinG = ROSHelper::ToOdometry(odom);
  odomIinG.header.stamp = ros::Time(sys->state->time);
  odomIinG.header.frame_id = "global";
  odomIinG.child_frame_id = "imu";

  // Finally set the covariance in the message (in the order position then orientation as per ros convention)
  // TODO: this currently is an approximation since this should actually evolve over our propagation period
  // TODO: but to save time we only propagate the mean and not the uncertainty, but maybe we should try to prop the covariance?
  vector<shared_ptr<Type>> var_pq, var_v;
  var_pq.push_back(sys->state->imu->pose()->p());
  var_pq.push_back(sys->state->imu->pose()->q());
  var_v.push_back(sys->state->imu->v());
  Matrix<double, 6, 6> covariance_posori = StateHelper::get_marginal_covariance(sys->state, var_pq);
  Matrix<double, 6, 6> covariance_linang = pow(op->est->imu->sigma_w, 2) * Matrix<double, 6, 6>::Identity();
  covariance_linang.block(0, 0, 3, 3) = StateHelper::get_marginal_covariance(sys->state, var_v);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      odomIinG.pose.covariance[6 * r + c] = covariance_posori(r, c);
      odomIinG.twist.covariance[6 * r + c] = (isnan(covariance_linang(r, c))) ? 0 : covariance_linang(r, c);
    }
  }

  // Finally, publish the resulting odometry message
  pub_imu_odom.publish(odomIinG);

  // Publish TF
  publish_tf();
}

void ROSPublisher::publish_tf() {
  // Publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
  tf::StampedTransform trans = ROSHelper::Pose2TF(sys->state->imu->pose(), false);
  trans.frame_id_ = "global";
  trans.child_frame_id_ = "imu";
  mTfBr->sendTransform(trans);

  // Loop through each sensor calibration and publish it
  if (sys->state->op->cam->enabled) {
    for (const auto &calib : sys->state->cam_extrinsic) {
      tf::StampedTransform trans_calib = ROSHelper::Pose2TF(calib.second, true);
      trans_calib.frame_id_ = "imu";
      trans_calib.child_frame_id_ = "cam" + to_string(calib.first);
      mTfBr->sendTransform(trans_calib);
    }
  }

  if (sys->state->op->gps->enabled) {
    for (const auto &calib : sys->state->gps_extrinsic) {
      tf::StampedTransform trans_calib = ROSHelper::Pos2TF(calib.second, true);
      trans_calib.frame_id_ = "imu";
      trans_calib.child_frame_id_ = "gps" + to_string(calib.first);
      mTfBr->sendTransform(trans_calib);
    }
  }

  if (sys->state->op->wheel->enabled) {
    tf::StampedTransform trans_calib = ROSHelper::Pose2TF(sys->state->wheel_extrinsic, true);
    trans_calib.frame_id_ = "imu";
    trans_calib.child_frame_id_ = "wheel";
    mTfBr->sendTransform(trans_calib);
  }

  // Publish clone poses
  int clone_count = 0;
  for (const auto &C : sys->state->clones) {
    tf::StampedTransform trans = ROSHelper::Pose2TF(C.second, false);
    trans.frame_id_ = "global";
    trans.child_frame_id_ = "c" + to_string(clone_count++);
    mTfBr->sendTransform(trans);
  }
}

void ROSPublisher::publish_state() {
  // Transform the path history if GNSS is initialized
  if (sys->state->op->gps->enabled && sys->up_gps->initialized && !traj_in_enu) {
    // Transform individual pose stored in the path
    for (auto &pose : path_imu) {
      pose = ROSHelper::ToENU(pose, sys->state->trans_WtoE->value());
    }
    // We only transform the trajectory once.
    traj_in_enu = true;
  }

  // Create pose of IMU (note we use the bag time)
  geometry_msgs::PoseWithCovarianceStamped poseIinM = ROSHelper::ToPoseCov(sys->state->imu->value().block(0, 0, 7, 1));
  poseIinM.header.stamp = ros::Time(sys->state->time);
  poseIinM.header.seq = seq_imu;
  poseIinM.header.frame_id = "global";

  // Finally set the covariance in the message (in the order position then orientation as per ros convention)
  vector<shared_ptr<Type>> statevars;
  statevars.push_back(sys->state->imu->pose()->p());
  statevars.push_back(sys->state->imu->pose()->q());
  Matrix<double, 6, 6> covariance_posori = StateHelper::get_marginal_covariance(sys->state, statevars);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      poseIinM.pose.covariance[6 * r + c] = covariance_posori(r, c);
    }
  }
  pub_imu_pose.publish(poseIinM);

  //=========================================================
  //=========================================================

  // Append to our pose vector
  geometry_msgs::PoseStamped posetemp;
  posetemp.header = poseIinM.header;
  posetemp.pose = poseIinM.pose.pose;
  path_imu.push_back(posetemp);

  // Create our path (IMU)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arrIMU;
  arrIMU.header.stamp = ros::Time::now();
  arrIMU.header.seq = seq_imu;
  arrIMU.header.frame_id = "global";
  for (int i = 0; i < (int)path_imu.size(); i += floor((double)path_imu.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(path_imu.at(i));
  }
  pub_imu_path.publish(arrIMU);

  // Move them forward in time
  seq_imu++;

  // Publish TF
  publish_tf();
}

void ROSPublisher::publish_cam_features() {

  // Check if we have subscribers
  if (pub_cam_msckf.getNumSubscribers() == 0 && pub_cam_slam.getNumSubscribers() == 0 && pub_cam_line.getNumSubscribers() == 0)
    return;

  // Get our good MSCKF features
  vector<Vector3d> feats_msckf = sys->up_cam->get_used_msckf();
  sensor_msgs::PointCloud2 cloud = ROSHelper::ToPointcloud(feats_msckf, "global");
  pub_cam_msckf.publish(cloud);

  // Get our good SLAM features
  vector<Vector3d> feats_slam = sys->state->get_features_SLAM();
  sensor_msgs::PointCloud2 cloud_SLAM = ROSHelper::ToPointcloud(feats_slam, "global");
  pub_cam_slam.publish(cloud_SLAM);
  
  // Get our line feature
  vector<Matrix<double, 6, 1>> feats_line = sys->up_cam->get_used_lines();
  // cout << " THe publish lines num is " << feats_line.size() << endl;
  visualization_msgs::Marker cloud_line = ROSHelper::ToMarker(feats_line, "global");
  pub_cam_line.publish(cloud_line);
}

void ROSPublisher::visualize() {
  // Return if we have not inited
  if (!sys->state->initialized)
    return;
  
  // publish state
  publish_state();

  // publish camera
  if (sys->state->op->cam->enabled)
    publish_cam_features();
}

void ROSPublisher::publish_cam_images(vector<int> cam_ids) {
  for (auto cam_id : cam_ids)
    publish_cam_images(cam_id);
}

void ROSPublisher::publish_cam_images(int cam_id) {
  // Publish image at cam 0 rate and have all the images from each camera
  if (cam_id != 0)
    return;

  for (int i = 0; i < op->est->cam->max_n; i++) {
    // skip if no subscriber
    if (pub_cam_image.at(i).getNumSubscribers() == 0)
      continue;

    // skip if this is larger id pair of stereo
    if (op->est->cam->stereo_pairs.find(i) != op->est->cam->stereo_pairs.end() && i > op->est->cam->stereo_pairs.at(i))
      continue;

    // Create our message & Publish
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    pub_cam_image.at(i).publish(cv_bridge::CvImage(header, "bgr8", sys->up_cam->get_track_img(i)).toImageMsg());
  }
}

void ROSPublisher::publish_gps(GPSData gps, bool isGeodetic) {

  // Visualize GPS measurements when we have datum
  if (sys->gps_datum.hasNaN())
    return;

  // Convert from a geodetic WGS-84 coordinated to East-North-Up
  if (isGeodetic)
    gps.meas = MathGPS::GeodeticToEnu(gps.meas, sys->gps_datum);

  // Now put the measurement in publisher
  geometry_msgs::PoseStamped poseGPSinENU;
  poseGPSinENU.header.stamp = ros::Time::now();
  poseGPSinENU.header.seq = seq_gps[gps.id];
  poseGPSinENU.header.frame_id = "global";
  poseGPSinENU.pose.position.x = gps.meas(0);
  poseGPSinENU.pose.position.y = gps.meas(1);
  poseGPSinENU.pose.position.z = gps.meas(2);
  poseGPSinENU.pose.orientation.x = 0.0;
  poseGPSinENU.pose.orientation.y = 0.0;
  poseGPSinENU.pose.orientation.z = 0.0;
  poseGPSinENU.pose.orientation.w = 1.0;
  pub_gps_pose[gps.id].publish(poseGPSinENU);

  // Append to our poses vector and create GPS path
  path_gps[gps.id].push_back(poseGPSinENU);
  nav_msgs::Path arrGPS;
  arrGPS.header.stamp = ros::Time::now();
  arrGPS.header.seq = seq_gps[gps.id];
  arrGPS.header.frame_id = "global";
  arrGPS.poses = path_gps[gps.id];
  pub_gps_path[gps.id].publish(arrGPS);

  // move sequence forward
  seq_gps[gps.id]++;
}