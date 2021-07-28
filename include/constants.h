#ifndef CONSTANTS
#define CONSTANTS
#include "common.h"

namespace utils {
namespace Constants {
template <typename T>
const Eigen::Matrix<T, 3, 1> gravity_v = Eigen::Matrix<T, 3, 1>::UnitZ() * 9.80665;

template <typename T>
constexpr T pi_v = static_cast<T>(3.1415926535897932);

template <typename T>
constexpr T deg2rad_v = static_cast<T>(3.1415926535897932 / 180.0);

template <typename T>
constexpr T rad2deg_v = static_cast<T>(180.0 / 3.1415926535897932);

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
Eigen::IOFormat OctaveFmt(4, 0, ", ", ";\n", "", "", "[", "]");
Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");

}  // namespace Constants

template <typename T, int M, int N = 1>
auto zeros = Eigen::Matrix<T, M, N>::Zero();

template <typename T, int M, int N = 1>
auto ones = Eigen::Matrix<T, M, N>::Ones();

template <typename T>
using StdVector_t = std::vector<T, Eigen::aligned_allocator<T>>;

}  // namespace utils

#endif  // CONSTANTS
