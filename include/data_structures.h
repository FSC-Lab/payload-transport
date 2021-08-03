#ifndef DATA_STRUCTURES
#define DATA_STRUCTURES

#include "common.h"

namespace utils {
struct Pose {
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Pose(const Eigen::Vector3d& t, const Eigen::Quaterniond& r) : position_(t), orientation_(r){};

  Pose() = default;

  Pose(const Pose& other) : position_(other.position_), orientation_(other.orientation_) {}

  Pose& operator=(const Pose& other) {
    Pose tmp(other);
    swap(*this, tmp);
    return *this;
  }

  Pose(Pose&& other) noexcept : Pose() { swap(*this, other); }

  Pose& operator=(Pose&& other) {
    Pose tmp(std::move(other));
    swap(*this, tmp);
    return *this;
  }

  const Eigen::Vector3d& position() const { return position_; }
  Eigen::Vector3d& position() { return position_; }
  const Eigen::Quaterniond& orientation() const { return orientation_; }
  Eigen::Quaterniond& orientation() { return orientation_; }

  friend void swap(Pose& lhs, Pose& rhs) {
    lhs.position_.swap(rhs.position_);
    lhs.orientation_.coeffs().swap(rhs.orientation_.coeffs());
  }

  friend std::ostream& operator<<(std::ostream& os, const Pose& pose) {
    const auto& p = pose.position();
    const auto& q = pose.orientation();
    os << std::setprecision(4) << "Position: [" << p.x() << " " << p.y() << " " << p.z() << "]\nOrientation: [" << q.w()
       << " " << q.x() << " " << q.y() << " " << q.z() << "]";
    return os;
  }

  static Pose fromMsg(const geometry_msgs::Pose& other) {
    Pose res;
    res.setFromMsg(other);
    return res;
  }

  static Pose fromMsg(geometry_msgs::Pose&& other) {
    Pose res;
    res.setFromMsg(other);
    return res;
  }

  void setFromMsg(const geometry_msgs::Pose& other) {
    position_ = Eigen::Vector3d::Map(&other.position.x);
    orientation_ = Eigen::Map<const Eigen::Quaterniond>(&other.orientation.x);
  }

  void setFromMsg(geometry_msgs::Pose&& other) {
    position_.swap(Eigen::Map<Eigen::Vector3d>(&other.position.x));
    orientation_.coeffs().swap(Eigen::Map<Eigen::Vector4d>(&other.orientation.x));
  }

  geometry_msgs::Pose toMsg() const {
    geometry_msgs::Pose res;
    Eigen::Vector3d::Map(&res.position.x) = position_;
    Eigen::QuaternionMapd(&res.orientation.x) = orientation_;
    return res;
  }
};

class Twist {
  Eigen::Vector3d linear_;
  Eigen::Vector3d angular_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Twist(const Eigen::Vector3d& v, const Eigen::Vector3d& w) : linear_(v), angular_(w){};

  Twist() = default;

  Twist(const Twist& other) : linear_(other.linear_), angular_(other.angular_) {}

  Twist& operator=(const Twist& other) {
    Twist tmp(other);
    swap(*this, tmp);
    return *this;
  }

  Twist(Twist&& other) noexcept : Twist() { swap(*this, other); }

  Twist& operator=(Twist&& other) {
    Twist tmp(std::move(other));
    swap(*this, tmp);
    return *this;
  }

  const Eigen::Vector3d& linear() const { return linear_; }
  Eigen::Vector3d& linear() { return linear_; }
  const Eigen::Vector3d& angular() const { return angular_; }
  Eigen::Vector3d& angular() { return angular_; }

  friend void swap(Twist& lhs, Twist& rhs) {
    lhs.linear_.swap(rhs.linear_);
    lhs.angular_.swap(rhs.angular_);
  }

  friend std::ostream& operator<<(std::ostream& os, const Twist& twist) {
    const auto& v = twist.linear();
    const auto& w = twist.angular();
    os << std::setprecision(4) << "Linear: [" << v.x() << " " << v.y() << " " << v.z() << "]\nAngular: ["
       << " " << w.x() << " " << w.y() << " " << w.z() << "]";
    return os;
  }

  static Twist fromMsg(const geometry_msgs::Twist& other) {
    Twist res;
    res.setFromMsg(other);
    return res;
  }

  static Twist fromMsg(geometry_msgs::Twist&& other) {
    Twist res;
    res.setFromMsg(other);
    return res;
  }

  void setFromMsg(const geometry_msgs::Twist& other) {
    linear_ = Eigen::Vector3d::Map(&other.linear.x);
    angular_ = Eigen::Vector3d::Map(&other.angular.x);
  }

  void setFromMsg(geometry_msgs::Twist&& other) {
    linear_.swap(Eigen::Map<Eigen::Vector3d>(&other.linear.x));
    angular_.swap(Eigen::Map<Eigen::Vector3d>(&other.angular.x));
  }

  geometry_msgs::Twist toMsg() const {
    geometry_msgs::Twist res;
    Eigen::Vector3d::Map(&res.linear.x) = linear_;
    Eigen::Vector3d::Map(&res.angular.x) = angular_;
    return res;
  }
};

class RigidBody {
  Pose pose_;
  Twist twist_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RigidBody(const Pose& pose, const Twist& twist) : pose_(pose), twist_(twist) {}

  RigidBody() = default;

  RigidBody(const RigidBody& other) : pose_(other.pose_), twist_(other.twist_) {}

  RigidBody& operator=(const RigidBody& other) {
    RigidBody tmp(other);
    swap(*this, tmp);
    return *this;
  }

  RigidBody(RigidBody&& other) noexcept : RigidBody() { swap(*this, other); }

  RigidBody& operator=(RigidBody&& other) {
    RigidBody tmp(std::move(other));
    swap(*this, tmp);
    return *this;
  }

  const Pose& pose() const { return pose_; }
  Pose& pose() { return pose_; }
  const Twist& twist() const { return twist_; }
  Twist& twist() { return twist_; }

  friend void swap(RigidBody& lhs, RigidBody& rhs) {
    swap(lhs.pose_, rhs.pose_);
    swap(lhs.twist_, rhs.twist_);
  }
};

class ThrustAndAttitudeTarget {
  double thrust_;
  Eigen::Quaterniond orientation_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ThrustAndAttitudeTarget(double thrust, const Eigen::Quaterniond& orientation)
      : thrust_(thrust), orientation_(orientation) {}

  ThrustAndAttitudeTarget() = default;

  ThrustAndAttitudeTarget(const ThrustAndAttitudeTarget& other)
      : thrust_(other.thrust_), orientation_(other.orientation_) {}

  ThrustAndAttitudeTarget& operator=(const ThrustAndAttitudeTarget& other) {
    ThrustAndAttitudeTarget tmp(other);
    swap(*this, tmp);
    return *this;
  }

  ThrustAndAttitudeTarget(ThrustAndAttitudeTarget&& other) noexcept : ThrustAndAttitudeTarget() { swap(*this, other); }

  ThrustAndAttitudeTarget& operator=(ThrustAndAttitudeTarget&& other) {
    ThrustAndAttitudeTarget tmp(std::move(other));
    swap(*this, tmp);
    return *this;
  }

  double thrust() const { return thrust_; }
  double& thrust() { return thrust_; }
  const Eigen::Quaterniond& orientation() const { return orientation_; }
  Eigen::Quaterniond& orientation() { return orientation_; }

  friend void swap(ThrustAndAttitudeTarget& lhs, ThrustAndAttitudeTarget& rhs) {
    using std::swap;
    swap(lhs.thrust_, rhs.thrust_);
    lhs.orientation_.coeffs().swap(rhs.orientation_.coeffs());
  }
};
}  // namespace utils

#endif  // DATA_STRUCTURES
