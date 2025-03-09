
#ifndef VIW_ROSPUBLISHER_H
#define VIW_ROSPUBLISHER_H

#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <memory>
#include <vector>

namespace ros {
class Publisher;
class NodeHandle;
} // namespace ros
namespace image_transport {
class Publisher;
}
namespace tf {
class TransformBroadcaster;
}

namespace viw {

class SystemManager;
struct GPSData;
class State;
struct Options;
class ROSPublisher {
public:

  /// ROS message publisher
  ROSPublisher(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<SystemManager> sys, std::shared_ptr<Options> op);

  ~ROSPublisher(){};

  /// Visualize state, camera & LiDAR features
  void visualize();

  /// Publish current IMU state (pose, ang/lin velocities)
  void publish_imu();

  /// Publish the gps measurements
  void publish_gps(GPSData gps, bool isGeodetic);

  /// Publish the active tracking image
  void publish_cam_images(std::vector<int> cam_ids);
  void publish_cam_images(int cam_id);

private:
  /// Publish the current state
  void publish_state();

  /// Publish current camera features
  void publish_cam_features();

  /// Publish TFs
  void publish_tf();

  /// Global node handler
  std::shared_ptr<ros::NodeHandle> nh;

  /// Core application of the filter system
  std::shared_ptr<SystemManager> sys;

  /// Options
  std::shared_ptr<Options> op;

  // Our publishers
  std::shared_ptr<tf::TransformBroadcaster> mTfBr;
  std::vector<image_transport::Publisher> pub_cam_image;
  ros::Publisher pub_imu_pose, pub_imu_odom, pub_imu_path, pub_cam_msckf, pub_cam_slam, pub_cam_line;
  std::vector<ros::Publisher> pub_gps_pose, pub_gps_path;

  // For path viz
  unsigned int seq_imu = 0;
  std::vector<unsigned int> seq_gps;
  std::vector<geometry_msgs::PoseStamped> path_imu;
  std::vector<std::vector<geometry_msgs::PoseStamped>> path_gps;
  bool traj_in_enu = false;
};
} // namespace viw

#endif // VIW_ROSPUBLISHER_H
