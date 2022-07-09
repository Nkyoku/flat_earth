/**
 * @file wgs84.hpp
 * @author Fujii Naomichi (nkyoku@gmail.com)
 * @copyright Copyright (c) 2022 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Eigen/Dense>

namespace flat_earth {

class Wgs84 {
public:
    /// 地球楕円体の赤道半径 [m]
    static constexpr double A = 6378137.0;

    /// 地球楕円体の扁平率
    static constexpr double F = 1.0 / 298.257223563;

    /// 短軸の半径
    static constexpr double B = A * (1 - F);

    /// 緯度 [deg]
    /// 正数は北緯、負数は南緯を表す。
    double latitude = 0.0;

    /// 経度 [deg]
    /// 正数は東経、負数は西経を表す。
    double longitude = 0.0;

    /// 楕円体高 [m]
    double altitude = 0.0;

    Wgs84(void) {}

    Wgs84(double latitude, double longitude, double altitude = 0.0) : latitude(latitude), longitude(longitude), altitude(altitude) {}

    explicit Wgs84(const Eigen::Vector3d &xyz);

    Eigen::Vector3d xyz(void) const;

    Eigen::Vector3d normal(void) const;
};

} // namespace flat_earth
