#ifndef WAYPOINTMANAGER
#define WAYPOINTMANAGER
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include <sensor_msgs/NavSatFix.h>
#include "common.h"

namespace utils {

class WaypointManager {
  utils::StdVector_t<Eigen::Vector3d> waypoints_;
  std::vector<double> velocities_;

  utils::StdVector_t<Eigen::Vector3d> headings_;

  ros::NodeHandle nh_;
  ros::Subscriber wp_sub_;
  ros::ServiceClient param_client_;

  const std::string wp_sub_key_;
  const std::string param_sub_key_;
  int wp_index_;

  mavros_msgs::WaypointList wps_;

 public:
  const utils::StdVector_t<Eigen::Vector3d>& waypoints() { return waypoints_; }

  auto gcsWaypointNum() { return wps_.waypoints.size(); }

  auto waypointNum() { return waypoints_.size(); }
  auto waypointIndex() { return wp_index_; }

  WaypointManager(const std::string& base_name)
      : nh_("~"),
        wp_sub_key_(base_name + "/mission/waypoints"),
        param_sub_key_(base_name + "/param/get"),
        wp_index_(0) {
    wp_sub_ = nh_.subscribe<mavros_msgs::WaypointList>(
        wp_sub_key_, 1, [&](const mavros_msgs::WaypointListConstPtr& msg) { wps_ = std::move(*msg); });

    param_client_ = nh_.serviceClient<mavros_msgs::ParamGet>(param_sub_key_);
  };

  bool getVelocities(double backup_speed = 4.0) {
    if (gcsWaypointNum() > 0) {
      if (nh_.getParam("/custom/nav/velocities", velocities_)) {
        if (velocities_.size() == gcsWaypointNum() - 1) {
          ROS_INFO("Got path-following velocities from parameters!");
          return true;
        }
      }
      velocities_.resize(gcsWaypointNum() - 1);
      std::fill(velocities_.begin(), velocities_.end(), backup_speed);
      ROS_INFO("Using fallback speed: %8.4f for path-following velocities", backup_speed);
      return true;
    }
    return false;
  }

  void computePath(const Eigen::Vector3d& reference_position) {
    waypoints_.clear();
    waypoints_.reserve(wps_.waypoints.size());
    using std::abs;
    for (const auto& it : wps_.waypoints) {
      if (it.command != 16 && it.frame != 3) {
        continue;
      }
      if (abs(it.x_lat) < 90 && abs(it.y_long) < 180) {
        const Eigen::Vector3d lla(it.x_lat, it.y_long, it.z_alt + reference_position.z());
        auto curr_waypoint = utils::Geo::CoordsGlobalToLocal(lla, reference_position);

        waypoints_.push_back(curr_waypoint);

        if (waypoints_.size() > 1) {
          const auto& last_waypoint = waypoints_.back();
          const Eigen::Vector3d heading = curr_waypoint - last_waypoint;
          headings_.push_back(heading);
        }
      }
    }
  }

  Eigen::Vector3d computePositionError(const Eigen::Vector3d& vehicle_position, double& heading_angle) {
    using std::atan2;

    const auto& next_waypoint = waypoints_[wp_index_ + 1];
    const Eigen::Vector2d dist_to_next_waypoint = (next_waypoint - vehicle_position).head<2>();
    heading_angle = atan2(dist_to_next_waypoint.y(), dist_to_next_waypoint.x());
    const auto& curr_waypoint = waypoints_[wp_index_];
    const Eigen::Vector3d dist_to_last_waypoint = curr_waypoint - vehicle_position;
    const auto& curr_heading = headings_[wp_index_];
    const Eigen::Vector3d pos_error = dist_to_last_waypoint - curr_heading.dot(dist_to_last_waypoint) * curr_heading;

    int n_waypoints = waypoints_.size();
    ROS_INFO("waypoint %d : %d", wp_index_, n_waypoints);
    if (wp_index_ == n_waypoints - 1) {
      ROS_ERROR("Payload control mission complete!");
    } else {
      if (pos_error.squaredNorm() < 0.5) {
        wp_index_++;
      }
    }
    return pos_error;
  }

  Eigen::Vector3d computeVelocityError(const Eigen::Vector3d& vehicle_velocity) const {
    const auto& curr_heading = headings_[wp_index_];
    const auto& cruise_speed = velocities_[wp_index_];
    const Eigen::Vector3d desired_velocity = cruise_speed * curr_heading;
    const Eigen::Vector3d vel_err = desired_velocity - vehicle_velocity;
    return vel_err;
  }
};
}  // namespace utils

#endif  // WAYPOINTMANAGER