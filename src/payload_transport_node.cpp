#include "MavrosManager.h"
#include "common.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "std_srvs/SetBool.h"

int main(int argc, char** argv) {
  using namespace std::string_literals;
  ros::init(argc, argv, "SlungLoadTransportController");

  auto nh = ros::NodeHandle("~");
  auto controller_param_ns = nh.param("controller_param_ns", "ctrl"s);
  ctl::PathFollowingControllerParams params(controller_param_ns);

  auto sys_model_param_ns = nh.param("sys_model_param_ns", "mass_geometry"s);
  auto feedback_source_topics = nh.param("feedback_source_topics", std::map<std::string, std::string>{});
  mdl::MultirotorPayloadDynamics sys_model(sys_model_param_ns, feedback_source_topics);

  auto motor_model_param_ns = nh.param("motor_model_param_ns", "motor"s);
  auto motor_coeffs = nh.param("/motor/coeffs", std::vector<double>{0.0, 0.3});
  auto motor_saturation_thrust = nh.param("/motor/saturation_thrust", 100);
  mdl::MotorModel motor_model(motor_coeffs, motor_saturation_thrust);

  ctl::PathFollowingController ctrl(sys_model, motor_model, params);

  auto mavros_ns = nh.param("mavros_ns", "/mavros"s);
  utils::MavrosManager mavros_man(mavros_ns);

  // auto controller_trigger_srv = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
  //     "/ctrl/run_controller", [&](auto req, auto res) {
  //       should_run_controller = req.data;
  //       res.success = true;
  //       res.message = "Controller is set to "s + (req.data ? "Run"s : "Stop"s);
  //       return true;
  //     });

  auto wp_man = utils::WaypointManager(mavros_ns);
  auto slow_rate = ros::Rate(0.5);
  ROS_INFO("Waiting for waypoints");
  auto timeout = ros::Duration(10);
  auto t0 = ros::Time::now();
  while (ros::ok() &&wp_man.gcsWaypointNum() == 0) {
    mavros_man.pullWaypoints();
    ros::spinOnce();
    slow_rate.sleep();
    if (ros::Time::now() - t0 > timeout) {
      ROS_INFO("Timeout! Failed to get waypoints!");
      return 1;
    }

  }

  ROS_INFO("Got %zu waypoints!", wp_man.gcsWaypointNum());

  for (const auto& it : wp_man.waypoints()) {
    std::cout << it.transpose() << "\n";
  }

  // const auto& home = wp_man.home_position();
  // ROS_INFO("Home position is [%8.4f, %8.4f, %8.4f]", home.x(), home.y(), home.z());
  const auto& geo = mavros_man.home_pos().geo;
  wp_man.computePath({geo.latitude, geo.longitude, geo.altitude});
  wp_man.getVelocities(mavros_man.getParam("MPC_XY_CRUISE", 4.0));
  return 0;
  // ROS_INFO("Computed paths and headings!");

  ros::Rate regular_rate(10);

  while (ros::ok() && !mavros_man.state().armed) {
    mavros_man.toggleArm(true);
    ros::spinOnce();
    slow_rate.sleep();
  }

  while (ros::ok() && mavros_man.state().mode != "AUTO.TAKEOFF") {
    mavros_man.takeoff();
    ros::spinOnce();
    slow_rate.sleep();
  }

  double altitude_start = sys_model.vehicle().pose().position().z();
  double altitude_end = mavros_man.getParam("MIS_TAKEOFF_ALT", 10.0);
  t0 = ros::Time::now();
  while (ros::ok() && mavros_man.state().mode == "AUTO.TAKEOFF") {
    auto curr_altitude = sys_model.vehicle().pose().position().z();
    if (curr_altitude - altitude_start > 1.0) {
      ROS_INFO("Takeoff in progress. Altitude: %8.4f", curr_altitude);

      if (abs(curr_altitude - altitude_end) / altitude_end < 1e-2) {
        ROS_INFO("Takeoff succeeded!");
      }
    } else {
      ROS_INFO("No altitude change detected!");
      if (ros::Time::now() - t0 > timeout) {
        ROS_INFO("Timeout! Failed to takeoff after 10 seconds! Exiting");
        ros::shutdown();
        return 0;
      }
    }
    ros::spinOnce();
    slow_rate.sleep();
  }

  while (ros::ok() && mavros_man.state().mode != "OFFBOARD") {
    // mavros_man.hoverAfterTakeoff();
    // mavros_man.setMode("OFFBOARD");
    ros::spinOnce();
    slow_rate.sleep();
  }

  return 0;
}