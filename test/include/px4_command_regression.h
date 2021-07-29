#ifndef PX4_COMMAND_REGRESSION
#define PX4_COMMAND_REGRESSION

#include <Eigen/Dense>

// Copied from px4_command/include/px4_command_utils.h, lines {299, 381}
// Original signature of this function is:
// px4_command::AttitudeReference ThrottleToAttitude(const Eigen::Vector3d& thr_sp, double yaw_sp)
void ThrottleToAttitude(const Eigen::Vector3d& thr_sp, double yaw_sp, Eigen::Vector3d& throttle_sp,
                        Eigen::Quaterniond& desired_att_q, double& desired_throttle,
                        Eigen::Vector3d& desired_attitude) {
  Eigen::Vector3d att_sp;
  att_sp[2] = yaw_sp;

  // desired body_z axis = -normalize(thrust_vector)
  Eigen::Vector3d body_x, body_y, body_z;

  double thr_sp_length = thr_sp.norm();

  // cout << "thr_sp_length : "<< thr_sp_length << endl;

  if (thr_sp_length > 0.00001f) {
    body_z = thr_sp.normalized();

  } else {
    // no thrust, set Z axis to safe value
    body_z = Eigen::Vector3d(0.0f, 0.0f, 1.0f);
  }

  // vector of desired yaw direction in XY plane, rotated by PI/2
  Eigen::Vector3d y_C = Eigen::Vector3d(-sinf(yaw_sp), cosf(yaw_sp), 0.0);

  if (fabsf(body_z(2)) > 0.000001f) {
    // desired body_x axis, orthogonal to body_z
    body_x = y_C.cross(body_z);

    // keep nose to front while inverted upside down
    if (body_z(2) < 0.0f) {
      body_x = -body_x;
    }

    body_x.normalize();

  } else {
    // desired thrust is in XY plane, set X downside to construct correct matrix,
    // but yaw component will not be used actually
    body_x = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
    body_x(2) = 1.0f;
  }

  // desired body_y axis
  body_y = body_z.cross(body_x);

  Eigen::Matrix3d R_sp;

  // fill rotation matrix
  for (int i = 0; i < 3; i++) {
    R_sp(i, 0) = body_x(i);
    R_sp(i, 1) = body_y(i);
    R_sp(i, 2) = body_z(i);
  }

  Eigen::Quaterniond q_sp(R_sp);

  // rotation_to_euler(R_sp, att_sp);

  // cout << "Desired euler [R P Y]: "<< att_sp[0]* 180/M_PI <<" [deg] " << att_sp[1]* 180/M_PI <<" [deg] "<< att_sp[2]*
  // 180/M_PI <<" [deg] "<< endl; cout << "Desired Thrust: "<< thr_sp_length<< endl;
  //    cout << "q_sp [x y z w]: "<< q_sp.x() <<" [ ] " << q_sp.y() <<" [ ] "<<q_sp.z() <<" [ ] "<<q_sp.w() <<" [ ]
  //    "<<endl; cout << "R_sp : "<< R_sp(0, 0) <<" " << R_sp(0, 1) <<" "<< R_sp(0, 2) << endl; cout << "     : "<<
  //    R_sp(1, 0) <<" " << R_sp(1, 1) <<" "<< R_sp(1, 2) << endl; cout << "     : "<< R_sp(2, 0) <<" " << R_sp(2, 1)
  //    <<" "<< R_sp(2, 2) << endl;

  throttle_sp = thr_sp;

  //期望油门
  desired_throttle = thr_sp_length;

  desired_att_q = q_sp;

  desired_attitude = att_sp;
}

Eigen::Vector3d thrustToThrottleLinear(const Eigen::Vector3d& thrust_sp, double slope, double intercept) {
  Eigen::Vector3d throttle_sp;
  // Linear motor model
  throttle_sp = (slope * thrust_sp.norm() + intercept) * thrust_sp.normalized();
  return throttle_sp;
}

#endif  // PX4_COMMAND_REGRESSION
