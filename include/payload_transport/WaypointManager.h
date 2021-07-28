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
  int wp_index_;

  ros::NodeHandle nh_;
  ros::Subscriber wp_sub_;
  ros::Subscriber gp_sub_;
  ros::ServiceClient wp_client_;
  ros::ServiceClient param_client_;

  const std::string gp_sub_key_;
  const std::string wp_sub_key_;

  const std::string param_sub_key_;
  const std::string wp_pull_key_;
  const std::string wp_clear_key_;

  mavros_msgs::WaypointList wps_;

  Eigen::Vector3d home_position_;

 public:
  const Eigen::Vector3d& home_position() const { return home_position_; }

  const utils::StdVector_t<Eigen::Vector3d>& waypoints() { return waypoints_; }

  auto gcsWaypointNum() { return wps_.waypoints.size(); }

  auto waypointNum() { return waypoints_.size(); }
  auto waypointIndex() { return wp_index_; }

  bool isHomePositionValid() { return !home_position_.isZero(1e-3); }

  WaypointManager(const std::string& base_name)
      : nh_("~"),
        wp_sub_key_(base_name + "/mission/waypoints"),
        wp_pull_key_(base_name + "/mission/pull"),
        gp_sub_key_(base_name + "/global_position/global"),
        param_sub_key_(base_name + "/param/get"),
        home_position_(utils::zeros<double, 3>),
        wp_index_(0) {
    wp_sub_ = nh_.subscribe<mavros_msgs::WaypointList>(
        wp_sub_key_, 1, [&](const mavros_msgs::WaypointListConstPtr& msg) { wps_ = std::move(*msg); });

    gp_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(gp_sub_key_, 1, [&](const sensor_msgs::NavSatFixConstPtr& msg) {
      home_position_ << msg->latitude, msg->longitude, msg->altitude;
    });

    wp_client_ = nh_.serviceClient<mavros_msgs::WaypointPull>(wp_pull_key_);
    param_client_ = nh_.serviceClient<mavros_msgs::ParamGet>(param_sub_key_);
  };

  bool getWaypoints() {
    mavros_msgs::WaypointPull srv;
    if (wp_client_.call(srv)) {
      if (srv.response.success && srv.response.wp_received > 0) {
        return true;
      } else {
        return false;
      }
    }
  }

  bool getVelocities() {
    if (isHomePositionValid() && gcsWaypointNum() > 0) {
      if (nh_.getParam("/custom/nav/velocities", velocities_)) {
        if (velocities_.size() == gcsWaypointNum() - 1) {
          ROS_INFO("Got path-following velocities from parameters!");
          return true;
        }
      } else {
        mavros_msgs::ParamGet srv;
        srv.request.param_id = "WPNAV_SPEED";
        if (param_client_.call(srv)) {
          if (srv.response.success) {
            auto& wpnav_speed = srv.response.value.real;
            velocities_.resize(gcsWaypointNum() - 1);
            std::fill(velocities_.begin(), velocities_.end(), wpnav_speed);
            ROS_INFO("Using WPNAV_SPEED: %8.4f as path-following velocities", wpnav_speed);
            return true;
          }
        }
      }
      auto backup_speed = 4.0;
      velocities_.resize(gcsWaypointNum() - 1);
      std::fill(velocities_.begin(), velocities_.end(), backup_speed);
      ROS_INFO("Using fallback speed: %8.4f for path-following velocities", backup_speed);
      return true;
    }
  }

  void computePath() {
    waypoints_.clear();
    waypoints_.reserve(wps_.waypoints.size());
    using std::abs;
    for (const auto& it : wps_.waypoints) {
      if (it.command != 16 || it.frame != 3) {
        continue;
      }
      if (abs(it.x_lat) < 90 && abs(it.y_long) < 180) {
        const Eigen::Vector3d lla(it.x_lat, it.y_long, it.z_alt + home_position_.z());
        auto wp = utils::Geo::CoordsGlobalToLocal(lla, home_position_);

        waypoints_.push_back(wp);
      }
    }
  }

  void computeHeadings() {
    if (headings_.size() > 0) {
      headings_.clear();
    }

    for (auto it = waypoints_.begin(); it != waypoints_.end() - 1; ++it) {
      const auto& curr_waypoint = *it;
      const auto& next_waypoint = *(it + 1);
      const Eigen::Vector3d curr_heading = (next_waypoint - curr_waypoint).normalized();
      headings_.push_back(curr_heading);
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
    ROS_INFO("waypoint %d : %zu", wp_index_, waypoints_.size());
    if (wp_index_ == waypoints_.size() - 1) {
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