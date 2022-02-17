// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
#include <payload_transport_controller/internal/geo_functions.h>

#include <numeric>
namespace common {
namespace geo {

static constexpr auto kDeg2Rad =
    0.0174532925199432954743716805978692718781530857086181640625L;
static constexpr auto kRad2Deg =
    57.29577951308232286464772187173366546630859375L;

static const GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

Eigen::Vector3d transformENUandECEF(
    const Eigen::Ref<const Eigen::Vector3d>& vec,
    const Eigen::Ref<const Eigen::Vector3d>& map_origin, bool inverse) {
  // Don't forget to convert from degrees to radians

  const double lat = kDeg2Rad * (90.0 - map_origin.x());
  const double lon = kDeg2Rad * (90.0 + map_origin.y());
  /**
   * @brief Compute transform from ECEF to ENU:
   * http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
   * ϕ = latitude
   * λ = longitude
   * The rotation is composed by a counter-clockwise rotation over the Z-axis
   * by an angle of 90 + λ followed by a counter-clockwise rotation over the
   * east-axis by an angle of 90 - ϕ. R =
   *      [-sinλ         cosλ         0.0
   *      -cosλ*sinϕ   -sinλ*sinϕ    cosϕ
   *       cosλ*cosϕ    sinλ*cosϕ    sinϕ   ]
   * [East, North, Up] = R * [∂x, ∂y, ∂z]
   * where both [East, North, Up] and [∂x, ∂y, ∂z] are local coordinates
   * relative to map origin.
   */

  const Eigen::Quaterniond R =
      Eigen::Quaterniond({-lat, Eigen::Vector3d::UnitX()}) *
      Eigen::Quaterniond({-lon, Eigen::Vector3d::UnitZ()});

  return (inverse ? R.inverse() : R) * vec;
}
Eigen::Vector3d CoordsGlobalToLocal(
    const Eigen::Ref<const Eigen::Vector3d>& target_lla,
    const Eigen::Ref<const Eigen::Vector3d>& center) {
  // current gps -> curent ECEF
  Eigen::Vector3d center_ecef;
  earth.Forward(center[0], center[1], center[2], center_ecef[0], center_ecef[1],
                center_ecef[2]);

  // goal gps -> goal ECEF
  Eigen::Vector3d goal_ecef;
  earth.Forward(target_lla[0], target_lla[1], target_lla[2], goal_ecef[0],
                goal_ecef[1], goal_ecef[2]);

  // get ENU offset from ECEF offset
  const Eigen::Vector3d ecef_offset = goal_ecef - center_ecef;
  const Eigen::Vector3d enu_offset =
      transformENUandECEF(ecef_offset, center, false);

  return enu_offset;
}

Eigen::Vector3d GetCentralCoordinate(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>>& coords) {
  auto sz = coords.size();
  if (sz == 1) {
    return coords.back();
  }

  double inv_sz = 1.0 / sz;
  const Eigen::Vector3d tmp =
      inv_sz * std::accumulate(
                   coords.begin(), coords.end(), Eigen::Vector3d::Zero().eval(),
                   [](const Eigen::Ref<const Eigen::Vector3d>& sum,
                      const Eigen::Ref<const Eigen::Vector3d>& it) {
                     using std::sin;
                     using std::cos;
                     const auto lat = it.x() * kDeg2Rad;
                     const auto lon = it.y() * kDeg2Rad;
                     const Eigen::Vector3d res =
                         sum + Eigen::Vector3d(cos(lat) * cos(lon),
                                               cos(lat) * sin(lon), sin(lat));
                     return res;
                   });

  const auto central_lon = std::atan2(tmp.y(), tmp.x());
  const auto central_sqrt = tmp.head<2>().norm();
  const auto central_lat = std::atan2(tmp.z(), central_sqrt);
  const auto central_alt =
      inv_sz * std::accumulate(
                   coords.begin(), coords.end(), 0.0,
                   [](double sum, const Eigen::Ref<const Eigen::Vector3d>& it) {
                     return sum += it.z();
                   });

  return Eigen::Vector3d(central_lat * kRad2Deg, central_lon * kRad2Deg,
                         central_alt);
}

bool IsValidLatLon(const Eigen::Ref<const Eigen::Vector3d>& lla) {
  return abs(lla.x()) < 90.0 && abs(lla.y()) < 180.0;
}
}  // namespace geo
}  // namespace common