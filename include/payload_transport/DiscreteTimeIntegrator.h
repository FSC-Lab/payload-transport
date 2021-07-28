#ifndef DISCRETETIMEINTEGRATOR
#define DISCRETETIMEINTEGRATOR
#include "common.h"

namespace utils {

template <typename T, int Xdim = Eigen::Dynamic>
class DiscreteTimeIntegrator {
  using ValueType = Eigen::Matrix<T, Xdim, 1>;

  const ValueType input_ub_;
  const ValueType input_lb_;
  const ValueType integral_ub_;
  const ValueType integral_lb_;
  ValueType value_;

  bool is_integral_saturated_;

 public:
  DiscreteTimeIntegrator(const ValueType& ic, T scalar_ub, T scalar_lb, T scalar_input_ub = 0, T scalar_input_lb = 0)
      : value_(ic),
        integral_lb_(ValueType::Constant(scalar_lb)),
        integral_ub_(ValueType::Constant(scalar_ub)),
        input_lb_(ValueType::Constant(scalar_input_lb)),
        input_ub_(ValueType::Constant(scalar_input_ub)),
        is_integral_saturated_(false){};

  DiscreteTimeIntegrator(const ValueType& ic, const ValueType& vector_ub, const ValueType& vector_lb,
                         const ValueType& vector_input_ub = ValueType::Zero(),
                         const ValueType& vector_input_lb = ValueType::Zero())
      : value_(ic),
        integral_lb_(vector_lb),
        integral_ub_(vector_ub),
        input_lb_(vector_input_lb),
        input_ub_(vector_input_ub),
        is_integral_saturated_(false){};

  /**
   * @brief Perform a single step Euler method update, corresponding to x_{k+1} = x_k + dt_k * f(x_k)
   *
   * @param dt Time step, usually t_{k+1} - t_k
   * @param input Input function value at step k, or f(x_k)
   * @return ValueType& The updated integral value, or x_{k+1}. Return value is immutable!
   */
  const ValueType& integrateOneStep(T dt, const ValueType& input) {
    if (!input_ub_.isZero() && !input_ub_.isZero()) {
      value_ += dt * Clip(input, input_lb_, input_ub_);
    } else {
      value_ += dt * input;
    }
    if ((value_.array() > integral_ub_.array()).any() || (value_.array() < integral_lb_.array()).any()) {
      ClipInPlace(value_, integral_lb_, integral_ub_);
      is_integral_saturated_ = true;
    } else {
      is_integral_saturated_ = false;
    }
    return value_;
  }

  /**
   * @brief Gets whether the integrator is saturated.
   *
   * @return true
   * @return false
   */
  bool is_integral_saturated() const { return is_integral_saturated_; }

  /**
   * @brief Gets the current integral value
   *
   * @return const ValueType& A constant reference to the current integral value
   */
  const ValueType& value() const { return value_; }

  /**
   * @brief Rezeros integral value
   *
   */
  void resetToZero() { value_.setZero(); }
};
}  // namespace utils

#endif  // DISCRETETIMEINTEGRATOR
