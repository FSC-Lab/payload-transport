#ifndef MAVROSMANAGER
#define MAVROSMANAGER

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPull.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace utils {
class MavrosManager {
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Subscriber home_pos_sub_;
  ros::Publisher local_pos_pub_;
  ros::Time last_request_;

  std::map<std::string, ros::ServiceClient> clients_;
  std::string home_position_topic_;
  std::string arm_key_;
  std::string takeoff_key_;
  std::string setmode_key_;
  std::string param_get_key_;
  std::string paramset_key_;
  std::string wp_pull_key_;

  bool got_home_position_;

  double takeoff_altitude_;

  mavros_msgs::State state_;
  void StateCb(const mavros_msgs::StateConstPtr& msg) { state_ = *msg; }

  mavros_msgs::HomePosition home_pos_;
  void GlobalPositionCb(const mavros_msgs::HomePositionConstPtr& msg) {
    got_home_position_ = true;
    home_pos_ = *msg;
  }

  inline void ShowCallFailureMsg(const std::string& msg) { ROS_INFO("Failed to call service: %s", msg.c_str()); }

 public:
  MavrosManager(const std::string& base_topic)
      : nh_("~"),
        last_request_(ros::Time::now()),
        home_position_topic_(base_topic + "/home_position/home"),
        arm_key_(base_topic + "/cmd/arming"),
        takeoff_key_(base_topic + "/cmd/takeoff"),
        setmode_key_(base_topic + "/set_mode"),
        param_get_key_(base_topic + "/param/get"),
        paramset_key_(base_topic + "/param/set"),
        wp_pull_key_(base_topic + "/mission/pull") {
    state_sub_ = nh_.subscribe<mavros_msgs::State>(base_topic + "/state", 10, &MavrosManager::StateCb, this);
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    home_pos_sub_ =
        nh_.subscribe<mavros_msgs::HomePosition>(home_position_topic_, 10, &MavrosManager::GlobalPositionCb, this);
    clients_[arm_key_] = nh_.serviceClient<mavros_msgs::CommandBool>(arm_key_);
    clients_[takeoff_key_] = nh_.serviceClient<mavros_msgs::CommandTOL>(takeoff_key_);
    clients_[setmode_key_] = nh_.serviceClient<mavros_msgs::SetMode>(setmode_key_);
    clients_[param_get_key_] = nh_.serviceClient<mavros_msgs::ParamGet>(param_get_key_);
    clients_[paramset_key_] = nh_.serviceClient<mavros_msgs::ParamSet>(paramset_key_);
    clients_[wp_pull_key_] = nh_.serviceClient<mavros_msgs::WaypointPull>(wp_pull_key_);
  }

  bool toggleArm(bool arm_value) {
    auto success = false;
    if (state_.armed == arm_value) {
      ROS_INFO("Already %s", arm_value ? "Armed" : "Disarmed");
      return true;
    }
    mavros_msgs::CommandBool payload;
    payload.request.value = arm_value;
    if (!clients_[arm_key_].call(payload)) {
      ShowCallFailureMsg(arm_key_);
    } else {
      if (!payload.response.success) {
        ROS_INFO("FCU refused to arm!");
      } else {
        ROS_INFO("Vehicle armed");
        success = true;
      }
    }
    last_request_ = ros::Time::now();
    return success;
  }

  bool setMode(const std::string& mode) {
    auto success = false;
    if (state_.mode == mode) {
      ROS_INFO("Already in %s mode", mode.c_str());
      return true;
    }
    mavros_msgs::SetMode payload;
    payload.request.custom_mode = mode;
    if (!clients_[setmode_key_].call(payload)) {
      ShowCallFailureMsg(setmode_key_);
    } else {
      if (!payload.response.mode_sent) {
        ROS_INFO("FCU refused to set %s", mode.c_str());
      } else {
        ROS_INFO("Offboard enabled");
        success = true;
      }
    }
    last_request_ = ros::Time::now();
    return success;
  }

  bool takeoff() {
    auto success = false;
    if (got_home_position_) {
      mavros_msgs::CommandTOL payload;
      payload.request.latitude = home_pos_.geo.latitude;
      payload.request.longitude = home_pos_.geo.longitude;
      payload.request.altitude = 10;
      if (!clients_[takeoff_key_].call(payload)) {
        ShowCallFailureMsg(takeoff_key_);
      } else {
        if (!payload.response.success) {
          ROS_INFO("FCU refused to takeoff!");
        } else {
          ROS_INFO("Attempted takeoff!");

          success = true;
        }
      }
    } else {
      ROS_INFO("Cannot takeoff! Cannot get home position!");
    }
    last_request_ = ros::Time::now();
    return success;
  }

  bool pullWaypoints() {
    auto success = false;
    mavros_msgs::WaypointPull payload;
    if (!clients_[wp_pull_key_].call(payload)) {
      ShowCallFailureMsg(wp_pull_key_);
    } else {
      if (!payload.response.success) {
        ROS_INFO("FCU refused to send waypoints!");
      } else {
        ROS_INFO("Pulled waypoints!");
        success = true;
      }
    }
    return success;
  }

  double getParam(const std::string& param_id, double default_value) {
    mavros_msgs::ParamGet payload;
    payload.request.param_id = param_id;
    if (!clients_[param_get_key_].call(payload)) {
      ShowCallFailureMsg(param_get_key_);
    } else {
      if (!payload.response.success) {
        ROS_INFO("Failed to get parameter %s", param_id.c_str());
      } else {
        const auto& val = payload.response.value;
        ROS_INFO("Got %s with value %4.2f", param_id.c_str(), val.real);
        return val.real;
      }
    }
    return default_value;
  }

  int getParam(const std::string& param_id, int default_value) {
    mavros_msgs::ParamGet payload;
    payload.request.param_id = param_id;
    if (!clients_[param_get_key_].call(payload)) {
      ShowCallFailureMsg(param_get_key_);
    } else {
      if (!payload.response.success) {
        ROS_INFO("Failed to get parameter %s", param_id.c_str());
      } else {
        const auto& val = payload.response.value;
        ROS_INFO("Got %s with value %ld", param_id.c_str(), val.integer);
        return val.real;
      }
    }
    return default_value;
  }

  const mavros_msgs::State& state() const { return state_; }

  const mavros_msgs::HomePosition& home_pos() const { return home_pos_; }
};

#endif  // MAVROSMANAGER
}