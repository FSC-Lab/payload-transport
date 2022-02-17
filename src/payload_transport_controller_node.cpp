// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <thread>

#include "payload_transport_controller/controller_ros_client.h"

using namespace std::string_literals;

std::string getCmdOption(int argc, char** argv, const std::string& option,
                         const std::string& default_opt = "") {
  auto begin = argv;
  auto end = argv + argc;
  auto it = std::find(begin, end, option);
  if (it != end && ++it != end) {
    return std::string(*it);
  }
  return default_opt;
}

void arming(ros::NodeHandle& nh, bool value, const std::string& mavros_ns,
            ros::Rate& rate) {
  using mavros_msgs::CommandBool;
  auto arming_key = mavros_ns + "/cmd/arming"s;
  auto state_topic = mavros_ns + "/state"s;
  auto srv = nh.serviceClient<CommandBool>(arming_key);
  bool armed = false;
  auto timeout = ros::Duration(5.0 * rate.cycleTime().toSec());
  while (ros::ok() && !armed) {
    auto p_state =
        ros::topic::waitForMessage<mavros_msgs::State>(state_topic, nh, timeout);
    if (!p_state) {
      ROS_ERROR("Failed to query state on %s!", state_topic.c_str());
    } else {
      armed = p_state->armed;
    }

    CommandBool pld;
    pld.request.value = value;
    if (!srv.call(pld)) {
      ROS_ERROR("Could not call %s", arming_key.c_str());
    }
    rate.sleep();
  }
  ROS_INFO("Successfully armed!");
}

void setMode(ros::NodeHandle& nh, const std::string& target_mode,
             const std::string& mavros_ns, ros::Rate& rate) {
  using mavros_msgs::SetMode;
  auto setmode_key = mavros_ns + "/set_mode"s;
  auto state_topic = mavros_ns + "/state"s;
  auto srv = nh.serviceClient<SetMode>(setmode_key);
  auto mode = ""s;
  auto timeout = ros::Duration(5.0 * rate.cycleTime().toSec());
  while (ros::ok() && mode != target_mode) {
    auto p_state =
        ros::topic::waitForMessage<mavros_msgs::State>(state_topic, nh, timeout);
    if (!p_state) {
      ROS_WARN("Failed to query state on %s!", state_topic.c_str());
    } else {
      mode = p_state->mode;
    }

    SetMode pld;
    pld.request.custom_mode = target_mode;
    if (!srv.call(pld)) {
      ROS_ERROR("Could not call %s", setmode_key.c_str());
    }
    rate.sleep();
  }
  ROS_INFO("Successfully set to %s!", target_mode.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "payload_transport_controller_node");

  ros::NodeHandle nh;
  ros::CallbackQueue listen_queue;
  nh.setCallbackQueue(&listen_queue);

  const auto hw_threads = std::thread::hardware_concurrency();
  const auto n_threads = std::min(hw_threads != 0 ? hw_threads : 2u, 6u);

  ros::AsyncSpinner spinner(n_threads, &listen_queue);
  spinner.start();
  auto mavros_ns = getCmdOption(argc, argv, "--mavros-ns"s, "/mavros"s);

  ControllerRosClient controller_driver(nh, mavros_ns, 1);
  std::thread controller;
  ros::Rate rate(50);
  if (controller_driver.initialize()) {
    controller = std::move(controller_driver.spawnControlLoop(rate));
  }

  ros::Rate setmode_rate(1.0);
  std::thread setmode_thread(setMode, std::ref(nh), "OFFBOARD",
                             std::cref(mavros_ns), std::ref(setmode_rate));

  ros::Rate arming_rate(1.0);
  std::thread arming_thread(arming, std::ref(nh), true, std::cref(mavros_ns),
                            std::ref(arming_rate));
  arming_thread.join();
  setmode_thread.join();
  controller.join();
  ros::Rate landing_rate(0.5);

  setMode(nh, "AUTO.LAND"s, mavros_ns, landing_rate);
  ros::waitForShutdown();
  return 0;
}