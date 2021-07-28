#ifndef MULTIROTORPAYLOADDYNAMICS_H
#define MULTIROTORPAYLOADDYNAMICS_H
#include "common.h"

namespace mdl {

class MultirotorPayloadDynamics {
  const Eigen::Vector3d grav = utils::Constants::gravity_v<double>;

  utils::RigidBody vehicle_;
  utils::RigidBody payload_;

  Eigen::Vector3d paylod_rel_position_;
  Eigen::Vector3d payload_rel_vel_;
  Eigen::Vector2d cable_vector_r_;
  Eigen::Vector2d cable_velocity_v_;

  const double payload_mass_;
  const double vehicle_mass_;
  const double total_mass_;
  const int num_motors_;
  const double cable_length_;
  const double cable_squared_length_;

 public:
  using CableMappingMatrix = Eigen::Matrix<double, 3, 2>;
  MultirotorPayloadDynamics(int num_motors, double vehicle_mass, double payload_mass, double cable_length)
      : num_motors_(num_motors),
        payload_mass_(payload_mass),
        vehicle_mass_(vehicle_mass),
        total_mass_(payload_mass + vehicle_mass),
        cable_length_(cable_length),
        cable_squared_length_(cable_length * cable_length){};

  CableMappingMatrix B_matrix() const {
    auto cable_z_comp_sq = (cable_squared_length_ - cable_vector_r_.squaredNorm());
    CableMappingMatrix B;
    if (cable_z_comp_sq > 1e-10) {
      auto cable_z_comp = sqrt(cable_z_comp_sq);
      B.row(2) = cable_vector_r_.transpose() / cable_z_comp;
    } else {
      B.row(2).setConstant(0.1);
    }
  }

  Eigen::Vector2d setCableVectorR(const Eigen::Vector3d &paylod_rel_position) {
    cable_vector_r_ = paylod_rel_position.head<2>();
  };

  Eigen::Vector3d setCableVectorL(const Eigen::Vector2d &cable_vector_r) {
    auto cable_z_comp = sqrt(cable_squared_length_ - cable_vector_r.squaredNorm());
    paylod_rel_position_ << cable_vector_r, cable_z_comp;
  }

  const Eigen::Vector2d &cable_vector_r() const { return cable_vector_r_; }
  const Eigen::Vector2d &cable_velocity_v() const { return cable_velocity_v_; }

  const utils::RigidBody &vehicle() const { return vehicle_; }
  utils::RigidBody &vehicle() { return vehicle_; }
  const utils::RigidBody &payload() const { return payload_; }
  utils::RigidBody &payload() { return payload_; }

  int num_motors() const { return num_motors_; }
  double payload_mass() const { return payload_mass_; }
  double vehicle_mass() const { return vehicle_mass_; }
  double total_mass() const { return total_mass_; }
  double cable_length() const { return cable_length_; }
  double cable_squared_length() const { return cable_squared_length_; }
};
}  // namespace mdl
#endif