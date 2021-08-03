#ifndef PATHFOLLOWINGCONTROLLER
#define PATHFOLLOWINGCONTROLLER
#include "common.h"

namespace ctl {

struct PathFollowingControllerParams {
  double err_p_xy;
  double err_p_z;
  double vel_p_xy;
  double vel_p_z;
  double Kpv_xy;
  double Kpv_z;
  double kL;

  double ude_weight_xy;
  double ude_weight_z;

  double err_vel_bounds_xy;
  double err_vel_bounds_z;

  double integral_bounds_xy;
  double integral_bounds_z;

  double err_filter_weight;
  double vel_filter_weight;

  double max_tilt_angle;

  bool init_ude_integration_active;
  bool init_error_filtering_active;

  Eigen::Vector3d err_p() const { return {err_p_xy, err_p_xy, err_p_z}; }
  Eigen::Vector3d vel_p() const { return {vel_p_xy, vel_p_xy, vel_p_z}; }
  Eigen::Vector3d ude_weight() const { return {ude_weight_xy, ude_weight_xy, ude_weight_z}; }
  Eigen::Vector3d integral_bounds() const { return {integral_bounds_xy, integral_bounds_xy, integral_bounds_z}; }
  Eigen::Vector3d err_vel_bounds() const { return {err_vel_bounds_xy, err_vel_bounds_xy, err_vel_bounds_z}; }

  PathFollowingControllerParams(const std::string &base_namespace = ""s) {
    ros::NodeHandle nh;
    err_p_xy = nh.param(base_namespace + "/err_p_xy", 1.0);
    err_p_z = nh.param(base_namespace + "/err_p_z", 2.0);
    vel_p_xy = nh.param(base_namespace + "/vel_p_xy", 0.5);
    vel_p_z = nh.param(base_namespace + "/vel_p_z", 0.5);
    ude_weight_xy = nh.param(base_namespace + "/ude_weight_xy", 0.0);
    ude_weight_z = nh.param(base_namespace + "/ude_weight_z", 0.0);
    err_vel_bounds_xy = nh.param(base_namespace + "/err_vel_bounds_xy", 0.3);
    err_vel_bounds_z = nh.param(base_namespace + "/err_vel_bounds_z", 0.3);
    integral_bounds_xy = nh.param(base_namespace + "/integral_bounds_xy", 0.5);
    integral_bounds_z = nh.param(base_namespace + "/integral_bounds_z", 0.5);
    err_filter_weight = nh.param(base_namespace + "/err_filter_weight", 1.0);
    vel_filter_weight = nh.param(base_namespace + "/vel_filter_weight", 1.0);
    max_tilt_angle = nh.param(base_namespace + "/max_tilt_angle", 45.0);
    init_error_filtering_active = nh.param(base_namespace + "/init/error_filtering_active", true);
    init_ude_integration_active = nh.param(base_namespace + "/init/ude_integration_active", true);
  }
};
class PathFollowingController {
  // Integral term in UDE
  utils::DiscreteTimeIntegrator<double, 3> ud_integrator_;
  const mdl::MultirotorPayloadDynamics &sys_mdl_;
  mdl::MotorModel motor_mdl_;
  PathFollowingControllerParams params_;

  Eigen::Vector3d err_p_;
  Eigen::Vector3d vel_p_;
  Eigen::Vector3d ude_weight_;
  double motor_saturation_thrust_;
  double err_filter_weight_;
  double vel_filter_weight_;
  double max_tilt_angle_;
  bool is_ude_integration_active_;
  bool is_error_filtering_active_;

  Eigen::Vector3d filterError(const Eigen::Vector3d &pos_error) {
    using std::sqrt;
    double filter_factor = 1.0 / sqrt(err_filter_weight_ + pos_error.squaredNorm());
    return pos_error * filter_factor;
  }

  Eigen::Vector3d filterVelocity(const Eigen::Vector3d &vel_error) {
    using std::sqrt;
    double filter_factor = 1.0 / sqrt(err_filter_weight_ + vel_error.squaredNorm());
    return vel_error * filter_factor;
  }

  Eigen::Vector3d get_ude_estimate() {
    const mdl::MultirotorPayloadDynamics::CableMappingMatrix B = sys_mdl_.B_matrix();
    return ude_weight_.cwiseProduct(sys_mdl_.payload_mass() * B * sys_mdl_.cable_velocity_v() +
                                    sys_mdl_.vehicle_mass() * sys_mdl_.vehicle().twist().linear() +
                                    ud_integrator_.value());
  }

 public:
  PathFollowingController(mdl::MultirotorPayloadDynamics &sys_mdl, mdl::MotorModel &motor_mdl,
                          const PathFollowingControllerParams &params)
      : ud_integrator_(utils::zeros<double, 3>, params_.integral_bounds(), -params_.integral_bounds(),
                       params_.err_vel_bounds(), -params_.err_vel_bounds()),
        sys_mdl_(sys_mdl),
        motor_mdl_(motor_mdl),
        err_p_(params.err_p()),
        vel_p_(params.vel_p()),
        ude_weight_(params.ude_weight()),
        err_filter_weight_(params.err_filter_weight),
        vel_filter_weight_(params.vel_filter_weight),
        max_tilt_angle_(params.max_tilt_angle),
        is_ude_integration_active_(params.init_ude_integration_active),
        is_error_filtering_active_(params.init_error_filtering_active){};

  utils::ThrustAndAttitudeTarget runController(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error,
                                               double yaw_sp, double dt) {
    Eigen::Vector3d path_and_velocity_error;
    if (is_error_filtering_active_) {
      path_and_velocity_error =
          err_p_.cwiseProduct(filterVelocity(pos_error)) + vel_p_.cwiseProduct(filterVelocity(vel_error));
    } else {
      path_and_velocity_error = err_p_.cwiseProduct(pos_error) + vel_p_.cwiseProduct(vel_error);
    }
    path_and_velocity_error = utils::Clip(path_and_velocity_error, 0.5);  // constraint the error

    // calculate the ud_integrator_ term of the UDE
    if (is_ude_integration_active_) {
      ud_integrator_.integrateOneStep(dt, -path_and_velocity_error);
    } else {
      ud_integrator_.resetToZero();
    }
    // calcualte total control force
    const Eigen::Vector3d ude_estimate = get_ude_estimate();
    const Eigen::Vector3d grav_compensation = sys_mdl_.total_mass() * utils::Constants::gravity_v<double>;

    Eigen::Vector3d thrust_setpoint_per_motor =
        (path_and_velocity_error - ude_estimate + grav_compensation) / sys_mdl_.num_motors();
    motor_mdl_.limitThrust(thrust_setpoint_per_motor, max_tilt_angle_);

    const Eigen::Vector3d throttle_setpoint = motor_mdl_(thrust_setpoint_per_motor);
    // throttle_sp = px4_command_utils::thrustToThrottleLinear(thrust_sp, motor_slope, motor_intercept);
    // publish auxiliary state
    utils::ThrustAndAttitudeTarget res;
    thrustToAttitudeSetpoint(throttle_setpoint, yaw_sp, res.orientation(), res.thrust());

    return res;
  }

  bool &is_ude_integration_active() { return is_ude_integration_active_; }
  bool is_ude_integration_active() const { return is_ude_integration_active_; }

  const mdl::MultirotorPayloadDynamics &system_model() const { return sys_mdl_; }

  static void thrustToAttitudeSetpoint(const Eigen::Vector3d &thrust_setpoint, const double yaw_setpoint,
                                       Eigen::Quaterniond &attitude_setpoint, double &throttle_setpoint) {
    using std::abs;
    using std::cos;
    using std::sin;

    // Machine Epsilon
    constexpr double kMchEps = std::numeric_limits<double>::epsilon();

    // Rotation matrix
    throttle_setpoint = thrust_setpoint.norm();

    Eigen::Matrix3d attitude_mat;
    if (throttle_setpoint > kMchEps) {
      // Set attitude_mat[:, 2] to unit vector aligned with thrust_setpoint
      attitude_mat.col(2) = thrust_setpoint / throttle_setpoint;
    } else {
      // thrust has 0 norm. Set atitude_mat[:, 2] to [0; 0; 1]
      attitude_mat.col(2) = Eigen::Vector3d::Zero();
    }

    // vector of desired yaw direction
    const Eigen::Vector3d heading_vector(-sin(yaw_setpoint), cos(yaw_setpoint), 0.0);

    // Check magnitude and sign of attitude_mat[2, 2] for
    // in case commanded thrust is fully lateral or upside down
    const double m_22 = attitude_mat(2, 2);
    if (abs(m_22) > kMchEps) {
      // desired body_x axis, orthogonal to body_z
      float flip = (m_22 < 0.0) ? -1.0 : 1.0;
      attitude_mat.col(0) = flip * heading_vector.cross(attitude_mat.col(2));
      attitude_mat.col(0).normalize();

    } else {
      // Desired thrust is fully lateral, set x-axis down
      // yaw setpoint is discarded
      attitude_mat.col(0) = Eigen::Vector3d::UnitZ();
    }

    // body-frame y-axis is simply tne x-axis cross z-axis, normalized
    attitude_mat.col(1) = attitude_mat.col(2).cross(attitude_mat.col(0));
    attitude_mat.col(1).normalize();

    // ROS uses quaternion internally, so convert the matrix to quaternion
    attitude_setpoint = Eigen::Quaterniond(attitude_mat);
  }
};

}  // namespace ctl

#endif  // PATHFOLLOWINGCONTROLLER
