// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef PATH_FOLLOWING_CONTROLLER_H
#define PATH_FOLLOWING_CONTROLLER_H

#include <Eigen/Dense>
#include <iostream>

#define DBG_MAT(m)                                                 \
  do {                                                             \
    std::cout << "EIGEN_EXPRESSION: " #m << " HAS_VALUE:\n"        \
              << (m).format({4, 0, ", ", ";\n", "", "", "[", "]"}) \
              << "\nEND_OF_DEBUG_MESSAGE\n";                       \
  } while (0)

class IntegralTerm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d value;

  explicit IntegralTerm(const Eigen::Vector3d& _value) : value(_value) {}

  void setWeight(double weight) { weight_ = std::max(0.0, weight); }

  void setBounds(const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
    if (lb.isApprox(ub) || (lb.array() >= ub.array()).any()) {
      bounds_valid_ = false;
      return;
    }
    lb_ = lb;
    ub_ = ub;
    bounds_valid_ = true;
  }

  void setBounds(double lb, double ub) {
    setBounds(Eigen::Vector3d::Constant(lb), Eigen::Vector3d::Constant(ub));
  }

  inline double weight() const { return weight_; }
  inline bool bounds_valid() const { return bounds_valid_; }
  inline const Eigen::Vector3d& ub() const { return ub_; }
  inline const Eigen::Vector3d& lb() const { return lb_; }

 private:
  bool bounds_valid_{false};
  double weight_{0.0};
  Eigen::Vector3d ub_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d lb_{Eigen::Vector3d::Zero()};
};

class PayloadTransportController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PayloadTransportController(
      double uav_mass, double pld_mass,
      const Eigen::Vector3d& init_disturbance = Eigen::Vector3d::Zero());

  ~PayloadTransportController();

  bool setPath(const Eigen::Vector3d& begin, const Eigen::Vector3d& end);

  bool run(double dt);

  void setCruiseSpeed(double cruise_speed);

  void setRadiusOfAcceptance(double radius_of_acceptance);

  void setDivergenceTimeout(double divergence_timeout);

  template <typename T>
  inline void setSaturationCb(T cb) {
    saturation_cb_ = std::move(cb);
  }

  template <typename T>
  inline void setUdeBounds(T&& lb, T&& ub) {
    ude_.setBounds(std::forward<T>(lb), std::forward<T>(ub));
  }

  inline void setUdeWeight(double weight) { ude_.setWeight(weight); }

  inline bool reached_endpoint() const { return reached_endpoint_; }

  inline const Eigen::Vector3d& thrust_sp() const { return thrust_sp_; }
  inline double yaw_sp() const { return yaw_sp_; }

  // Vehicle states
  Eigen::Vector3d uav_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d uav_vel{Eigen::Vector3d::Zero()};
  double yaw{0.0};

  Eigen::Vector3d pld_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pld_vel{Eigen::Vector3d::Zero()};

  Eigen::Vector3d kp_pos;
  Eigen::Vector3d kp_vel;

 private:
  // Internal Parameters
  double uav_mass_;
  double pld_mass_;
  double system_mass_;
  Eigen::Vector3d system_weight_;
  std::function<Eigen::Vector3d(const Eigen::Ref<const Eigen::Vector3d>&)>
      saturation_cb_;
  Eigen::Vector3d thrust_sp_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_error_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_error_{Eigen::Vector3d::Zero()};
  IntegralTerm ude_{Eigen::Vector3d::Zero()};

  // Setpoints
  double cruise_speed_{0.0};
  double yaw_sp_{0.0};
  Eigen::Vector3d path_begin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d path_end_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d course_{Eigen::Vector3d::Zero()};

  // Controller status checking
  Eigen::Vector3d to_end_{Eigen::Vector3d::Zero()};
  double dist_to_end_sq_{0.0};
  double radius_acceptance_sq_{0.0};
  double divergence_timeout_{0.0};
  double divergence_time_{0.0};
  double dist_to_end_sq_last_{0.0};
  bool reached_endpoint_{false};
  bool diverge_flag_{false};

  void updateUdeIntegral(const Eigen::Vector3d& value, double dt);
  void checkDistToPathEndPoint(double dt);

  Eigen::Vector3d udeEstimate();
};

#endif  // PATH_FOLLOWING_CONTROLLER_H
