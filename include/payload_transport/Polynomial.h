#ifndef POLYNOMIAL
#define POLYNOMIAL
#include "common.h"
namespace utils {
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

    for (int i = 0; i < degree_; i++) {
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
#endif  // POLYNOMIAL
