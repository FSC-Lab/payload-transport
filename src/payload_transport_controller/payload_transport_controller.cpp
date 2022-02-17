// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "payload_transport_controller/payload_transport_controller.h"

PayloadTransportController::PayloadTransportController(
    double uav_mass, double pld_mass, const Eigen::Vector3d &init_disturbance)
    : uav_mass_(uav_mass),
      pld_mass_(pld_mass),
      system_mass_(uav_mass + pld_mass),
      system_weight_(system_mass_ * 9.80665 * Eigen::Vector3d::UnitZ()),
      ude_(init_disturbance) {}

PayloadTransportController::~PayloadTransportController() = default;

void PayloadTransportController::setCruiseSpeed(double cruise_speed) {
  cruise_speed_ = std::max(cruise_speed, 0.0);
}

void PayloadTransportController::setRadiusOfAcceptance(double radius) {
  radius = std::max(radius, 0.0);
  radius_acceptance_sq_ = radius * radius;
}

void PayloadTransportController::setDivergenceTimeout(
    double divergence_timeout) {
  divergence_timeout_ = std::max(divergence_timeout, 0.0);
}

void PayloadTransportController::checkDistToPathEndPoint(double dt) {
  if (diverge_flag_) {
    return;
  }

  if (reached_endpoint_) {
    divergence_time_ = 0.0;
    return;
  }

  // If the distance towards the next waypoint increases, then the vehicle
  // is moving AWAY from the next waypoint.
  if (dist_to_end_sq_ < dist_to_end_sq_last_) {
    // Vehicle is moving TOWARDS next waypoint normally
    divergence_time_ = 0.0;
  } else {
    // Increment duration of vehicle moving AWAY from the next waypoint
    divergence_time_ += dt;
  }

  dist_to_end_sq_last_ = dist_to_end_sq_;
  // Vehicle is moving AWAY from the next waypoint for too long. Raise an
  // error and exit
  diverge_flag_ = divergence_time_ > divergence_timeout_;

  if (diverge_flag_) {
    // Upon detecting divergence, reset path end / destination to current
    // position
    path_end_ = uav_pos;
  }
}

Eigen::Vector3d PayloadTransportController::udeEstimate() {
  using std::max;
  using std::sqrt;

  Eigen::Vector3d res = ude_.weight() * (system_mass_ * uav_vel +
                                         pld_mass_ * pld_vel + ude_.value);
  if (ude_.bounds_valid()) {
    res = res.cwiseMax(ude_.lb()).cwiseMin(ude_.ub());
  }
  return res;
}

bool PayloadTransportController::setPath(const Eigen::Vector3d &begin,
                                         const Eigen::Vector3d &end) {
  path_begin_ = begin;
  path_end_ = end;
  const Eigen::Vector3d dist_between_wps = end - begin;
  if (dist_between_wps.squaredNorm() > std::numeric_limits<double>::epsilon()) {
    course_ = dist_between_wps.normalized();
    reached_endpoint_ = false;
    return true;
  }
  return false;
}

void PayloadTransportController::updateUdeIntegral(
    const Eigen::Vector3d &integrand, double dt) {
  ude_.value += integrand * dt;
  if (ude_.bounds_valid()) {
    ude_.value = ude_.value.cwiseMax(ude_.lb()).cwiseMin(ude_.ub());
  }
}

bool PayloadTransportController::run(double dt) {
  to_end_ = path_end_ - uav_pos;
  dist_to_end_sq_ = to_end_.squaredNorm();
  // If close to next waypoint within radius of acceptance, increment waypoint
  // counter and continue
  if (dist_to_end_sq_ < radius_acceptance_sq_) {
    reached_endpoint_ = true;
  }
  checkDistToPathEndPoint(dt);

  if (reached_endpoint_ || diverge_flag_) {
    // Pause at path endpoint upon reaching it. Distance away from endpoint IS
    // error; Nonzero velocity IS error.
    pos_error_ = -to_end_;
    vel_error_ = uav_vel;
    yaw_sp_ = yaw;
  } else {
    const Eigen::Vector3d to_last_wp = uav_pos - path_begin_;

    pos_error_ = to_last_wp - to_last_wp.dot(course_) * course_;
    vel_error_ = uav_vel - cruise_speed_ * course_;
  }
  Eigen::Vector3d pos_and_velocity_error;
  if (saturation_cb_) {
    pos_and_velocity_error = kp_pos.cwiseProduct(saturation_cb_(pos_error_)) +
                             kp_vel.cwiseProduct(saturation_cb_(vel_error_));
  } else {
    pos_and_velocity_error =
        kp_pos.cwiseProduct(pos_error_) + kp_vel.cwiseProduct(vel_error_);
  }

  // calculate the ud_integrator_ term of the UDE
  updateUdeIntegral(pos_and_velocity_error, dt);
  thrust_sp_ = -pos_and_velocity_error - udeEstimate() + system_weight_;

  const double lat_thrust_xy_sq = pos_and_velocity_error.head<2>().squaredNorm();
  const double lat_thrust_z_sq =
      pos_and_velocity_error.z() * pos_and_velocity_error.z();

  if (lat_thrust_z_sq > 3.0 * lat_thrust_xy_sq) {
    yaw_sp_ = yaw;
  } else {
    yaw_sp_ = atan2(course_.y(), course_.x());
  }

  return !diverge_flag_;
}
