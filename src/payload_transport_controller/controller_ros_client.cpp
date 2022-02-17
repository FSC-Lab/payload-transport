// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "payload_transport_controller/controller_ros_client.h"

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>

#include <Eigen/Dense>

#include "payload_transport_controller/internal/geo_functions.h"

using namespace std::string_literals;
namespace arg = std::placeholders;

ControllerRosClient::ControllerRosClient(ros::NodeHandle &nh,
                                         const std::string &mavros_ns,
                                         std::size_t queue_size)
    : nh_(nh),
      mavros_ns_(mavros_ns),
      queue_size_(queue_size),
      sync_(queue_size) {}

ControllerRosClient::~ControllerRosClient() = default;

bool ControllerRosClient::initialize() {
  using Self = ControllerRosClient;
  const std::string state_topic = mavros_ns_ + "/state"s;
  const auto &state_ptr = ros::topic::waitForMessage<mavros_msgs::State>(
      state_topic, nh_, ros::Duration(10.0));
  if (!state_ptr || !state_ptr->connected) {
    if (LogLevel::kError >= log_level_) {
      ROS_ERROR("Failed to connect to mavros. namespace is %s",
                mavros_ns_.c_str());
    }
    return false;
  }
  // Handle params
  ros::NodeHandle pnh("~");
  auto uav_mass = pnh.param("vehicle_mass"s, 1.5);
  auto pld_mass = pnh.param("payload_mass"s, 0.0);
  ctl_.reset(new PayloadTransportController(uav_mass, pld_mass));
  if (!ctl_) {
    if (LogLevel::kError >= log_level_) {
      ROS_ERROR("Failed to construct controller!");
    }
    return false;
  }

  log_level_ = static_cast<LogLevel>(pnh.param("log_level"s, 1));
  auto radius_acceptance = pnh.param("mission/radius_acceptance"s, 0.5);
  ctl_->setRadiusOfAcceptance(radius_acceptance);

  auto cruise_speed = pnh.param("mission/cruise_speed"s, 3.0);
  ctl_->setCruiseSpeed(cruise_speed);

  auto divergence_timeout = pnh.param("mission/divergence_timeout"s, 5.0);
  ctl_->setDivergenceTimeout(divergence_timeout);

  ROS_INFO("Mission: cruise speed %5.3f and radius of acceptance % 5.3f ",
           cruise_speed, radius_acceptance);

  auto weight = pnh.param("controller/filter_weight"s, 1.0);
  auto saturation_cb = [weight](const Eigen::Vector3d &x) -> Eigen::Vector3d {
    return x / std::sqrt(weight + x.squaredNorm());
  };
  ctl_->setSaturationCb(std::move(saturation_cb));

  auto ude_weight = pnh.param("controller/ude/weight"s, 0.1);
  ctl_->setUdeWeight(ude_weight);

  std::vector<double> lb, ub;
  if (pnh.getParam("controller/ude/lower_bound", lb) &&
      pnh.getParam("controller/ude/upper_bound", ub)) {
    if (ub.size() != 3 || ub.size() != 3) {
      ROS_ERROR(
          "Componentwise UDE lower / upper bounds must be 3-element arrays");
      return false;
    }

    const Eigen::Vector3d lower = Eigen::Map<const Eigen::Vector3d>(lb.data());
    const Eigen::Vector3d upper = Eigen::Map<const Eigen::Vector3d>(ub.data());
    if (lower.isApprox(upper) || (lower.array() >= upper.array()).any()) {
      ROS_ERROR("Invalid bounds!");
      return false;
    }
    ctl_->setUdeBounds(lower, upper);
  } else {
    double lower = pnh.param("controller/ude/lower_bound", -0.1);
    double upper = pnh.param("controller/ude/upper_bound", 0.1);
    if (lower >= upper) {
      ROS_ERROR("Invalid bounds!");
      return false;
    }
    ctl_->setUdeBounds(lower, upper);
  }
  ctl_->kp_pos.x() = ctl_->kp_pos.y() = pnh.param("controller/kp_pos_xy"s, 1.5);
  ctl_->kp_pos.z() = pnh.param("controller/kp_pos_z"s, 2.0);
  ctl_->kp_vel.x() = ctl_->kp_vel.y() = pnh.param("controller/kp_vel_xy"s, 0.25);
  ctl_->kp_vel.z() = pnh.param("controller/kp_vel_z"s, 0.25);

  auto saturation_thrust = pnh.param("safe/saturation_thrust"s, 30.0);
  auto max_tilt_angle = pnh.param("safe/max_tilt_angle"s, 0.5);
  auto offset = pnh.param("motor_model/offset"s, 0.0);
  auto scale = pnh.param("motor_model/scale"s, 0.0475);
  auto thrust_curve = [offset, scale](double x) { return offset + scale * x; };
  thrust_tracking_ctl_.reset(new ThrustTrackingController(
      saturation_thrust, max_tilt_angle, std::move(thrust_curve)));

  double max_climb_angle = pnh.param("mission/max_climb_angle"s, 45.0);
  static constexpr double kDeg2Rad = 0.017453292519943295;
  max_climb_ratio_sq_ =
      tan(kDeg2Rad * max_climb_angle) * tan(kDeg2Rad * max_climb_angle);
  const std::string waypoint_topic =
      pnh.param("mission/waypoint_topic"s, mavros_ns_ + "/mission/waypoints"s);
  const std::string home_topic =
      pnh.param("mission/home_topic"s, mavros_ns_ + "/home_position/home"s);

  Eigen::Vector3d home;
  Eigen::Vector3d home_lla;
  if (!getHome(home_topic, home, home_lla) ||
      !getWaypoints(waypoint_topic, home, home_lla)) {
    return false;
  }

  const Eigen::Vector3d curr_wp = mp_.front();
  mp_.pop();
  ctl_->setPath(curr_wp, mp_.front());
  // Handle Subscribes and advertises
  const auto pose_topic = mavros_ns_ + "/local_position/pose"s;
  const auto local_vel_topic = mavros_ns_ + "/local_position/velocity_local"s;
  const auto setpoint_topic = mavros_ns_ + "/setpoint_raw/attitude";
  local_pose_sub_.subscribe(nh_, pose_topic, queue_size_);
  local_vel_sub_.subscribe(nh_, local_vel_topic, queue_size_);
  sync_.connectInput(local_pose_sub_, local_vel_sub_);
  sync_.registerCallback(std::bind(&Self::dynamicsCb, this, arg::_1, arg::_2));

  cmd_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(setpoint_topic, 1);
  ROS_INFO("Successfully initialized controller!");
  return true;
}

void ControllerRosClient::dynamicsCb(
    const geometry_msgs::PoseStampedConstPtr &p_msg,
    const geometry_msgs::TwistStampedConstPtr &v_msg) {
  ctl_->uav_pos = Eigen::Map<const Eigen::Vector3d>(&p_msg->pose.position.x);
  ctl_->uav_vel = Eigen::Map<const Eigen::Vector3d>(&v_msg->twist.linear.x);

  auto &q = p_msg->pose.orientation;

  ctl_->yaw =
      atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool ControllerRosClient::getHome(const std::string &home_topic,
                                  Eigen::Ref<Eigen::Vector3d> home,
                                  Eigen::Ref<Eigen::Vector3d> home_lla) {
  // Get home position for computing local path
  auto home_ptr = ros::topic::waitForMessage<mavros_msgs::HomePosition>(
      home_topic, nh_, ros::Duration(10.0));
  if (!home_ptr) {
    if (LogLevel::kError >= log_level_) {
      ROS_ERROR("Failed to get home uav_pos!");
    }
    return false;
  }
  const auto &geo = home_ptr->geo;
  home = Eigen::Map<const Eigen::Vector3d>(&home_ptr->position.x);
  home_lla << geo.latitude, geo.longitude, geo.altitude;
  if (LogLevel::kInfo >= log_level_) {
    ROS_INFO_STREAM(
        "Got home position at: " << home_lla.transpose().format(fmt_));
  }

  return true;
}

bool ControllerRosClient::getWaypoints(
    const std::string &waypoint_topic, const Eigen::Ref<Eigen::Vector3d> &home,
    const Eigen::Ref<Eigen::Vector3d> &home_lla) {
  // Home position or takeoff location is always zeroth waypoint
  auto wp_ptr = ros::topic::waitForMessage<mavros_msgs::WaypointList>(
      waypoint_topic, nh_, ros::Duration(10.0));
  if (!wp_ptr || wp_ptr->waypoints.size() == 0u) {
    ROS_ERROR("Failed to get a nonzero number of waypoints on %s!",
              waypoint_topic.c_str());
    return false;
  }
  const auto &wps = wp_ptr->waypoints;
  if (LogLevel::kInfo >= log_level_) {
    ROS_INFO("Got %zu waypoints from mission", wps.size());
  }
  std::stringstream ss;
  mp_.push(home);
  ss << home.transpose().format(fmt_) << "\n";
  for (const auto &it : wps) {
    // Skip processing waypoints that are not NAV_WAYPOINT, or not defined in
    // FRAME_GLOBAL_REL_ALT

    if (it.command == 16 && it.frame == 6) {
      const Eigen::Vector3d lla(it.x_lat, it.y_long, home_lla.z() + it.z_alt);
      const Eigen::Vector3d new_wp =
          common::geo::CoordsGlobalToLocal(lla, home_lla);

      const auto &last_wp = mp_.back();
      if (!mp_.empty()) {
        const Eigen::Vector3d wp_dist = new_wp - last_wp;

        const double wp_vert_dist = abs(wp_dist.z());
        const double wp_horz_dist = wp_dist.head<2>().norm();
        if (wp_vert_dist / wp_horz_dist > max_climb_ratio_sq_) {
          const Eigen::Vector3d intermediate_wp(last_wp.x(), last_wp.y(),
                                                new_wp.z());
          mp_.push(intermediate_wp);
          ss << intermediate_wp.transpose().format(fmt_) << "\n";
        }
      }
      mp_.push(new_wp);
      ss << new_wp.transpose().format(fmt_) << "\n";
    }
  }
  if (LogLevel::kInfo >= log_level_) {
    ROS_INFO_STREAM("Waypoints are:\n" << ss.str());
  }
  return true;
}

void ControllerRosClient::controlLoop(ros::Rate &rate) {
  auto last_time = ros::Time::now();
  while (ros::ok()) {
    auto now = ros::Time::now();
    auto dt = now - last_time;
    if (!ctl_->run(dt.toSec())) {
      if (LogLevel::kError >= log_level_) {
        ROS_ERROR("Controller exited abnormally!");
      }
      return;
    }

    if (ctl_->reached_endpoint()) {
      ROS_INFO("Reached endpoint");
      const Eigen::Vector3d curr_waypoint = mp_.front();
      mp_.pop();
      if (mp_.empty()) {
        ROS_INFO("Mission ended successfully! Exiting!");
        return;
      } else {
        const Eigen::Vector3d next_waypoint = mp_.front();
        if (LogLevel::kInfo >= log_level_) {
          ROS_INFO_STREAM("Setting new path segment:\nFrom:\t"
                          << curr_waypoint.transpose().format(
                                 {2, 0, ", ", ";\n", "", "", "[", "]"})
                          << "\nTo:\t"
                          << next_waypoint.transpose().format(
                                 {2, 0, ", ", ";\n", "", "", "[", "]"}));
          ctl_->setPath(curr_waypoint, next_waypoint);
        }
      }
    }
    mavros_msgs::AttitudeTarget pld;
    Eigen::Map<Eigen::Quaterniond> orientation(&pld.orientation.x);
    std::tie(orientation, pld.thrust) =
        thrust_tracking_ctl_->run(ctl_->thrust_sp(), ctl_->yaw_sp());
    pld.header.stamp = ros::Time::now();
    cmd_pub_.publish(pld);
    last_time = now;
    rate.sleep();
  }
}

std::thread ControllerRosClient::spawnControlLoop(ros::Rate &rate) {
  std::thread controller(&ControllerRosClient::controlLoop, this,
                         std::ref(rate));
  return controller;
}

// namespace common
