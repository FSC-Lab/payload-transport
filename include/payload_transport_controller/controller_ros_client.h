// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef CONTROLLER_DRIVER_H
#define CONTROLLER_DRIVER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <queue>
#include <string>
#include <thread>

#include "payload_transport_controller/payload_transport_controller.h"
#include "payload_transport_controller/thrust_tracking_controller.h"

enum class LogLevel { kDebug = 0, kInfo = 1, kWarn = 2, kError = 3 };

class ControllerRosClient {
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      geometry_msgs::PoseStamped, geometry_msgs::TwistStamped>;

 public:
  ControllerRosClient(ros::NodeHandle &nh, const std::string &mavros_ns,
                      std::size_t queue_size);

  ~ControllerRosClient();

  bool initialize();

  void controlLoop(ros::Rate &rate);

  std::thread spawnControlLoop(ros::Rate &rate);

 private:
  void dynamicsCb(const geometry_msgs::PoseStampedConstPtr &p_msg,
                  const geometry_msgs::TwistStampedConstPtr &v_msg);
  bool getHome(const std::string &home_topic, Eigen::Ref<Eigen::Vector3d> home,
               Eigen::Ref<Eigen::Vector3d> home_lla);

  bool getWaypoints(const std::string &waypoint_topic,
                    const Eigen::Ref<Eigen::Vector3d> &home,
                    const Eigen::Ref<Eigen::Vector3d> &home_lla);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ros::NodeHandle nh_;
  std::string mavros_ns_;
  std::size_t queue_size_;
  LogLevel log_level_{LogLevel::kInfo};

  double max_climb_ratio_sq_;
  std::queue<
      Eigen::Vector3d,
      std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>
      mp_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> local_pose_sub_;
  message_filters::Subscriber<geometry_msgs::TwistStamped> local_vel_sub_;
  ros::Publisher cmd_pub_;
  std::unique_ptr<PayloadTransportController> ctl_;
  std::unique_ptr<ThrustTrackingController> thrust_tracking_ctl_;

  Eigen::IOFormat fmt_{4, 0, ", ", ";\n", "", "", "[", "]"};
};
#endif  // CONTROLLER_DRIVER_H
