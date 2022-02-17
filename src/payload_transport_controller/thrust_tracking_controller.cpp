// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "payload_transport_controller/thrust_tracking_controller.h"

ThrustTrackingController::ThrustTrackingController(double saturation_thrust,
                                                   double max_tilt_angle,
                                                   ThrustCurve &&motor_mdl)
    : saturation_thrust_(saturation_thrust),
      saturation_thrust_sq_(saturation_thrust * saturation_thrust),
      max_tilt_angle_(max_tilt_angle),
      max_tilt_ratio_(std::tan(max_tilt_angle)),
      motor_mdl_(std::move(motor_mdl)) {}

Eigen::Quaterniond ThrustTrackingController::vectorToOrientation(
    const Eigen::Vector3d &vector, double yaw) {
  using std::abs;
  using std::cos;
  using std::sin;

  Eigen::Matrix3d R;
  R.col(2) = vector;

  // Check magnitude and sign of attitude_mat[2, 2] for
  // in case commanded thrust is fully lateral or upside down
  const double R_22 = R.coeff(2, 2);
  if (abs(R_22) > std::numeric_limits<double>::epsilon()) {
    // vector of desired yaw direction
    const Eigen::Matrix<double, 3, 1> heading_vector(-sin(yaw), cos(yaw), 0.0);
    // desired body_x axis, orthogonal to body_z
    R.col(0) = (R_22 < 0.0 ? -heading_vector : heading_vector).cross(R.col(2));
    R.col(0).normalize();

  } else {
    // Desired thrust is fully lateral, set x-axis down
    // yaw setpoint is discarded
    R.col(0) = Eigen::Vector3d::UnitZ();
  }

  // body-frame y-axis is simply tne x-axis cross z-axis, normalized
  R.col(1) = R.col(2).cross(R.col(0));
  R.col(1).normalize();

  // ROS uses quaternion internally, so convert the matrix to quaternion
  Eigen::Quaterniond res(R);
  return res;
}

void ThrustTrackingController::limitThrust(Eigen::Vector3d &thrust_sp) const {
  using std::abs;
  using std::min;
  using std::sqrt;
  double max_lateral_thrust;
  if (thrust_sp.z() < saturation_thrust_) {
    // Lift does not saturate actuators, aim to deliver requested lift exactly
    // while scaling back lateral thrust
    max_lateral_thrust =
        sqrt(saturation_thrust_sq_ - thrust_sp.z() * thrust_sp.z());
  } else {
    // Lift alone saturates actuators, deliver as much lift as possible and no
    // lateral thrust
    thrust_sp.z() = saturation_thrust_;
    thrust_sp.head<2>().setZero();
    return;
  }
  if (max_tilt_angle_ > 0.0) {
    const double max_tilt_lateral_thrust = abs(thrust_sp.z()) * max_tilt_ratio_;
    max_lateral_thrust = min(max_lateral_thrust, max_tilt_lateral_thrust);
  }

  const double lateral_thrust_sqnorm = thrust_sp.head<2>().squaredNorm();
  if (lateral_thrust_sqnorm > max_lateral_thrust * max_lateral_thrust) {
    thrust_sp.head<2>() *= (max_lateral_thrust / sqrt(lateral_thrust_sqnorm));
  }
}

std::tuple<Eigen::Quaterniond, double> ThrustTrackingController::run(
    const Eigen::Vector3d &thrust_sp, const double yaw_sp) const {
  using std::sqrt;

  Eigen::Vector3d thrust_vector = thrust_sp;
  limitThrust(thrust_vector);
  const double thrust_sq = thrust_vector.squaredNorm();
  if (thrust_sq > std::numeric_limits<double>::epsilon()) {
    const double raw_thrust = sqrt(thrust_sq);
    thrust_vector /= raw_thrust;
    double thrust = motor_mdl_(raw_thrust);
    const Eigen::Quaterniond orientation =
        vectorToOrientation(thrust_vector, yaw_sp);
    return std::make_pair(orientation, thrust);
  } else {
    return std::make_pair(Eigen::Quaterniond::Identity(), 0.0);
  }
}