// Copyright (c) 2021 hs293go
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef GEO_FUNCTIONS_H
#define GEO_FUNCTIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/Geocentric.hpp>
#include <vector>
namespace common {
namespace geo {

template <typename T>
Eigen::Vector3d ToEigen(const T& pt) {
  Eigen::Vector3d res(pt.latitude, pt.longitude, pt.altitude);
  return res;
}

template <typename T>
void FromEigen(const Eigen::Ref<const Eigen::Vector3d>& src, T& dst) {
  dst.latitude = src.x();
  dst.longitude = src.y();
  dst.altitude = src.z();
}

Eigen::Vector3d transformENUandECEF(
    const Eigen::Ref<const Eigen::Vector3d>& vec,
    const Eigen::Ref<const Eigen::Vector3d>& map_origin, bool inverse);

Eigen::Vector3d CoordsGlobalToLocal(
    const Eigen::Ref<const Eigen::Vector3d>& target_lla,
    const Eigen::Ref<const Eigen::Vector3d>& center);

Eigen::Vector3d GetCentralCoordinate(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>>& coords);

bool IsValidLatLon(const Eigen::Ref<const Eigen::Vector3d>& lla);

}  // namespace geo
}  // namespace common
#endif  // GEO_FUNCTIONS_H
