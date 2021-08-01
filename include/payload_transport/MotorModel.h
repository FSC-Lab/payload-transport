#ifndef MOTORMODEL
#define MOTORMODEL
#include "Polynomial.h"
#include "common.h"

namespace mdl {

class MotorModel : utils::Polynomial {
  std::vector<double> coeffs_;
  double motor_saturation_thrust_;

 public:
  MotorModel(const std::vector<double> &coeffs, double motor_saturation_thrust)
      : utils::Polynomial(coeffs), motor_saturation_thrust_(motor_saturation_thrust){};

  void limitThrust(Eigen::Vector3d &thrust_setpoint, double max_tilt_angle = 0.0) {
    using std::abs;
    using std::min;
    using std::sqrt;
    using std::tan;

    const double &thrust_zcomp = thrust_setpoint[2];

    double max_thrust_xycomp;
    if (thrust_zcomp < motor_saturation_thrust_) {
      // Lift does not saturate actuators, aim to deliver requested lift exactly while scaling back lateral thrust
      max_thrust_xycomp =
          sqrt(motor_saturation_thrust_ * motor_saturation_thrust_ - thrust_zcomp * thrust_zcomp);
    } else {
      // Lift alone saturates actuators, deliver as much lift as possible and no lateral thrust
      thrust_setpoint[2] = motor_saturation_thrust_;
      thrust_setpoint.head<2>().setZero();
      return;
    }
;
    if (max_tilt_angle > 0.0) {
      const double max_lateral_thrust = tan(utils::deg2rad(max_tilt_angle));
      max_thrust_xycomp = min(max_lateral_thrust, max_thrust_xycomp);
    }

    const double thrust_xycomp_sq_norm = thrust_setpoint.head<2>().squaredNorm();

    if (thrust_xycomp_sq_norm > max_thrust_xycomp * max_thrust_xycomp) {
      const double thrust_xycomp_norm = sqrt(thrust_xycomp_sq_norm);
      const double thrust_rescale_factor = max_thrust_xycomp / thrust_xycomp_norm;
      thrust_setpoint.head<2>() *= thrust_rescale_factor;
    }
  }

  Eigen::Vector3d operator()(const Eigen::Vector3d &val) {
    double norm_val = val.norm();
    return Polynomial::operator()(norm_val) * val / norm_val;
  }
};
}  // namespace mdl

#endif  // MOTORMODEL
