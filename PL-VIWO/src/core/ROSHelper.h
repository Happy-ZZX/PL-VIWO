#ifndef VIW_ROSVISUALIZER_HELPER_H
#define VIW_ROSVISUALIZER_HELPER_H

#include <Eigen/Eigen>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace sensor_msgs;

namespace ov_core {
struct ImuData;
struct CameraData;
} // namespace ov_core
namespace ov_type {
struct PoseJPL;
struct Vec;
} // namespace ov_type
namespace pcl {
class PointXYZ;
template <class pointT> class PointCloud;
} // namespace pcl

namespace viw {
class State;
struct OptionsCamera;
struct WheelData;
struct GPSData;

/**
 * @brief Helper class that handles some common versions into and out of ROS formats
 */
class ROSHelper {

public:
  /// Collection of format converters
  static sensor_msgs::PointCloud2 ToPointcloud(const vector<Eigen::Vector3d> &feats, string frame_id);

  static tf::StampedTransform Pose2TF(const shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans);

  static tf::StampedTransform Pos2TF(const shared_ptr<ov_type::Vec> &pos, bool flip_trans);

  static ov_core::ImuData Imu2Data(const sensor_msgs::Imu::ConstPtr &msg);

  static bool Image2Data(const sensor_msgs::ImageConstPtr &msg, int cam_id, ov_core::CameraData &cam, shared_ptr<OptionsCamera> op);

  static bool Image2Data(const sensor_msgs::CompressedImageConstPtr &msg, int cam_id, ov_core::CameraData &cam, shared_ptr<OptionsCamera> op);

  static WheelData JointState2Data(const sensor_msgs::JointStateConstPtr &msg);

  static WheelData Odometry2Data(const nav_msgs::OdometryPtr &msg);

  static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> rosPC2pclPC(const sensor_msgs::PointCloud2ConstPtr &msg, int id);

  static GPSData PoseStamped2Data(const geometry_msgs::PoseStampedPtr &msg, int id, double noise);

  static GPSData NavSatFix2Data(const sensor_msgs::NavSatFixPtr &msg, int id);

  static GPSData NavSatFix2Data(const sensor_msgs::NavSatFixConstPtr &msg, int id);

  static nav_msgs::Odometry ToOdometry(Eigen::Matrix<double, 13, 1> state);

  static geometry_msgs::PoseWithCovarianceStamped ToPoseCov(Eigen::Matrix<double, 7, 1> state);

  static geometry_msgs::PoseStamped ToENU(geometry_msgs::PoseStamped pose, Eigen::Matrix<double, 7, 1> trans_WtoE);

  static visualization_msgs::Marker ToMarker(vector<Eigen::Matrix<double, 6, 1>> &lines, string frame_id);
};

} // namespace viw

#endif //VIW_ROSVISUALIZER_HELPER_H