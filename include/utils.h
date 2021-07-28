#ifndef UTILS_H
#define UTILS_H

#include "common.h"
#include "constants.h"

namespace utils {

namespace Geo {

template <typename Container1, typename Container2>
Eigen::Vector3d CoordsGlobalToLocal(const Container1& target_lla, const Container2& map_center) {
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

  // current gps -> curent ECEF
  Eigen::Vector3d current_ecef;
  earth.Forward(map_center[0], map_center[1], map_center[2], current_ecef[0], current_ecef[1], current_ecef[2]);

  // goal gps -> goal ECEF
  Eigen::Vector3d goal_ecef;
  earth.Forward(target_lla[0], target_lla[1], target_lla[2], goal_ecef[0], goal_ecef[1], goal_ecef[2]);

  // get ENU offset from ECEF offset
  const Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;
  const Eigen::Vector3d enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, map_center);

  return enu_offset;
}
}  // namespace Geo

template <typename Derived>
auto Clip(const Eigen::ArrayBase<Derived>& v, typename Derived::Scalar lb, typename Derived::Scalar ub) {
  return v.max(lb).min(ub);
}

template <typename Derived>
auto Clip(const Eigen::MatrixBase<Derived>& v, typename Derived::Scalar lb, typename Derived::Scalar ub) {
  return Clip(v.array(), lb, ub).matrix();
}

template <typename Derived>
auto Clip(const Eigen::ArrayBase<Derived>& v, typename Derived::Scalar sb) {
  return Clip(v, -sb, sb);
}

template <typename Derived>
auto Clip(const Eigen::MatrixBase<Derived>& v, typename Derived::Scalar sb) {
  return Clip(v.array(), sb).matrix();
}

template <typename Derived>
void ClipInPlace(Eigen::ArrayBase<Derived>& v, typename Derived::Scalar lb, typename Derived::Scalar ub) {
  v = Clip(v, lb, ub);
}

template <typename Derived>
void ClipInPlace(Eigen::MatrixBase<Derived>& v, typename Derived::Scalar lb, typename Derived::Scalar ub) {
  v = Clip(v.array(), lb, ub).matrix();
}

template <typename Derived>
void ClipInPlace(Eigen::ArrayBase<Derived>& v, typename Derived::Scalar sb) {
  v = Clip(v, sb);
}

template <typename Derived>
void ClipInPlace(Eigen::MatrixBase<Derived>& v, typename Derived::Scalar sb) {
  v = Clip(v.array(), sb).matrix();
}

template <typename DerivedA, typename DerivedB>
auto Clip(const Eigen::ArrayBase<DerivedA>& v, const Eigen::ArrayBase<DerivedB>& lb,
          const Eigen::ArrayBase<DerivedB>& ub) {
  return v.cwiseMax(lb).cwiseMin(ub);
}

template <typename DerivedA, typename DerivedB>
auto Clip(const Eigen::MatrixBase<DerivedA>& v, const Eigen::MatrixBase<DerivedB>& lb,
          const Eigen::MatrixBase<DerivedB>& ub) {
  return Clip(v.array(), lb.array(), ub.array()).matrix();
}

template <typename DerivedA, typename DerivedB>
auto Clip(const Eigen::ArrayBase<DerivedA>& v, const Eigen::ArrayBase<DerivedB>& sb) {
  return Clip(v, -sb, sb);
}

template <typename DerivedA, typename DerivedB>
auto Clip(const Eigen::MatrixBase<DerivedA>& v, const Eigen::MatrixBase<DerivedB>& sb) {
  return Clip(v.array(), sb.array()).matrix();
}

template <typename DerivedA, typename DerivedB>
void ClipInPlace(Eigen::ArrayBase<DerivedA>& v, const Eigen::ArrayBase<DerivedB>& lb,
                 const Eigen::ArrayBase<DerivedB>& ub) {
  v = Clip(v, lb, ub);
}

template <typename DerivedA, typename DerivedB>
void ClipInPlace(Eigen::MatrixBase<DerivedA>& v, const Eigen::MatrixBase<DerivedB>& lb,
                 const Eigen::MatrixBase<DerivedB>& ub) {
  v = Clip(v.array(), lb.array(), ub.array()).matrix();
}

template <typename DerivedA, typename DerivedB>
void ClipInPlace(Eigen::ArrayBase<DerivedA>& v, const Eigen::ArrayBase<DerivedB>& sb) {
  v = Clip(v, sb);
}

template <typename DerivedA, typename DerivedB>
void ClipInPlace(Eigen::MatrixBase<DerivedA>& v, const Eigen::MatrixBase<DerivedB>& sb) {
  v = Clip(v.array(), sb.array()).matrix();
}

/**
 * @brief Converts angle `in` from radians to degrees
 *
 * @param in input angle in radians
 * @return constexpr T input angle in degrees
 */
template <typename T>
constexpr T rad2deg(T in) {
  return Constants::rad2deg_v<T> * in;
}

/**
 * @brief Converts angle `in` from degrees to radians
 *
 * @param in input angle in degrees
 * @return constexpr T input angle in radians
 */
template <typename T>
constexpr T deg2rad(T in) {
  return Constants::deg2rad_v<T> * in;
}

/**
 * @brief Given the length of the hypotenuse and one leg, return the length of the other leg
 *
 * @param hypot length of the hypotenuse
 * @param leg length of one leg
 * @return constexpr T
 */
template <typename T>
constexpr T leg(T hypot, T leg) {
  using std::abs;
  using std::sqrt;
  return sqrt(abs(hypot * hypot - leg * leg));
}

template <typename T, typename U>
auto constexpr fastpow(T base, U exponent) {
  static_assert(std::is_integral<U>(), "exponent must be integral");
  return exponent == 0 ? 1 : base * fastpow(base, exponent - 1);
}

class Polynomial {
  std::vector<double> coeffs_;
  double degree_;

 public:
  // constructor
  Polynomial(const std::vector<double>& coeffs) : coeffs_(coeffs), degree_(coeffs.size()) {}

  double& coeff(int k) { return coeffs_[k]; }
  double coeff(int k) const { return coeffs_[k]; }
  const std::vector<double>& coeffs() const { return coeffs_; }
  std::vector<double>& coeffs() { return coeffs_; }

  template <typename T>
  typename std::enable_if_t<std::is_arithmetic<T>::value, T> operator()(T val) const {
    T power = 1;
    T res = 0;

    for (int i = 0; i <= degree_; i++) {
      res += coeffs_[i] * power;
      power *= val;
    }
    return res;
  }

  template <typename Derived>
  Eigen::Array<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> operator()(
      const Eigen::ArrayBase<Derived>& val) const {
    Eigen::Array<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> power = Eigen::ArrayBase<Derived>::Ones();
    Eigen::Array<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> res = Eigen::ArrayBase<Derived>::Zero();

    for (int i = 0; i <= degree_; i++) {
      res += coeffs_[i] * power;
      power *= val;
    }
    return res;
  }

  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> operator()(
      const Eigen::MatrixBase<Derived>& val) const {
    return (*this)(val.array()).matrix();
  }
};

}  // namespace utils
#endif