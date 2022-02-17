// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef THRUST_TRACKING_CONTROLLER_H
#define THRUST_TRACKING_CONTROLLER_H

#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <utility>

class ThrustTrackingController {
  double saturation_thrust_;
  double saturation_thrust_sq_;
  double max_tilt_angle_;
  double max_tilt_ratio_;

  using ThrustCurve = std::function<double(double)>;

  ThrustCurve motor_mdl_;

  static Eigen::Quaterniond vectorToOrientation(const Eigen::Vector3d &vector,
                                                double yaw = 0.0);

 public:
  ThrustTrackingController(double max_tilt_angle, double saturation_thrust,
                           ThrustCurve &&motor_mdl);

  void limitThrust(Eigen::Vector3d &thrust_sp) const;

  std::tuple<Eigen::Quaterniond, double> run(const Eigen::Vector3d &thrust_sp,
                                             const double yaw_sp) const;
};

#endif  // THRUST_TRACKING_CONTROLLER_H
