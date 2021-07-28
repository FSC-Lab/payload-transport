
#include <mavros/frame_tf.h>
#include "common.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "SlungLoadTransportController");

  auto nh = ros::NodeHandle("~");
  auto num_motors = nh.param("/mass_geometry/num_motors", 4);
  auto vehicle_mass = nh.param("/mass_geometry/vehicle_mass", 1.0);
  auto payload_mass = nh.param("/mass_geometry/payload_mass", 0.5);
  auto cable_length = nh.param("/mass_geometry/cable_length", 0.5);
  mdl::MultirotorPayloadDynamics model(num_motors, payload_mass, vehicle_mass, cable_length);

  auto pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 10,
      [&](const geometry_msgs::PoseStampedConstPtr& msg) { model.vehicle().pose() = utils::Pose::fromMsg(msg->pose); });

  auto wp_man = utils::WaypointManager("/mavros");

  wp_man.getWaypoints();
  auto slow_rate = ros::Rate(1);
  while (ros::ok() && (wp_man.gcsWaypointNum() == 0 || !wp_man.isHomePositionValid())) {
    ROS_INFO("Waiting for waypoints and home position...");

    ros::spinOnce();
    slow_rate.sleep();
  }
  ROS_INFO("Got %zu waypoints!", wp_man.gcsWaypointNum());
  const auto& home = wp_man.home_position();
  ROS_INFO("Home position is [%8.4f, %8.4f, %8.4f]", home.x(), home.y(), home.z());

  wp_man.computePath();
  wp_man.computeHeadings();
  ROS_INFO("Computed paths and headings!");

  auto regular_rate = ros::Rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    double heading = 0.0;
    const auto& error = wp_man.computePositionError(model.vehicle().pose().position(), heading);
    std::cout << error.transpose() << " "
              << model.vehicle().pose().position().transpose().format(utils::Constants::OctaveFmt) << " "
              << wp_man.waypoints()[wp_man.waypointIndex()].transpose().format(utils::Constants::OctaveFmt) << "\n";
    regular_rate.sleep();
  }
  return 0;
}