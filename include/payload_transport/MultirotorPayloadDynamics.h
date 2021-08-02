#ifndef MULTIROTORPAYLOADDYNAMICS
#define MULTIROTORPAYLOADDYNAMICS
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

  double payload_mass_;
  double vehicle_mass_;
  double total_mass_;
  int num_motors_;
  double cable_length_;
  double cable_squared_length_;

 public:
  using CableMappingMatrix = Eigen::Matrix<double, 3, 2>;
  MultirotorPayloadDynamics(const std::string &base_namespace = "mass_geometry"s) {
    ros::NodeHandle nh("~");
    num_motors_ = nh.param(base_namespace + "/num_motors", 4);
    vehicle_mass_ = nh.param(base_namespace + "/vehicle_mass", 1.0);
    payload_mass_ = nh.param(base_namespace + "/payload_mass", 0.5);
    cable_length_ = nh.param(base_namespace + "/cable_length", 0.5);
    total_mass_ = payload_mass_ + vehicle_mass_;
    cable_squared_length_ = cable_length_ * cable_length_;

  }

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
#endif  // MULTIROTORPAYLOADDYNAMICS
