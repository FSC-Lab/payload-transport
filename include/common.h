#ifndef COMMON
#define COMMON

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <numeric>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>

#include <mavros/frame_tf.h>
#include <GeographicLib/Geocentric.hpp>

#include "constants.h"
#include "utils.h"

#include "data_structures.h"
#include "payload_transport/DiscreteTimeIntegrator.h"
#include "payload_transport/MotorModel.h"
#include "payload_transport/MultirotorPayloadDynamics.h"
#include "payload_transport/PathFollowingController.h"
#include "payload_transport/WaypointManager.h"

namespace utils {
class Pose;
class Twist;
class RigidBody;
class MotorModel;

namespace Geo {
template <typename Container1, typename Container2>
Eigen::Vector3d CoordsGlobalToLocal(const Container1&, const Container2&);
}

}  // namespace utils

namespace ctl {
class PathFollowingController;
struct PathFollowingControllerParams;
}  // namespace ctl

namespace mdl {
class MotorModel;
class MultirotorPayloadDynamics;
}  // namespace mdl
#endif // COMMON
