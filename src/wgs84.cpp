/**
 * @file wgs84.cpp
 * @author Fujii Naomichi (nkyoku@gmail.com)
 * @copyright Copyright (c) 2022 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#include "wgs84.hpp"
#include <cmath>
#include <tuple>

namespace flat_earth {

static constexpr double PI = 3.14159265358979323846;

/**
 * @brief ラジアンから度へ変換する。
 * @param rad ラジアン [rad]
 * @return 度 [deg]
 */
static inline double rad2deg(double rad) {
    return rad * (180.0 / PI);
}

/**
 * @brief 度からラジアンへ変換する。
 * @param deg 度 [deg]
 * @return ラジアン [rad]
 */
static inline double deg2rad(double deg) {
    return deg * (PI / 180.0);
}

/**
 * @brief 直交座標系から楕円体座標系へ変換する。
 * @tparam T 楕円体座標系クラス
 * @param {x,y,z} 直交座標 [m]
 * @return {b,l,h} 楕円体座標(緯度 [rad], 経度 [rad], 楕円体高 [m])
 */
template<class T>
static inline std::tuple<double, double, double> xyz2blh(double x, double y, double z) {
    constexpr double A = T::A;
    constexpr double F = T::F;
    constexpr double E2 = 2.0 * F - F * F;
    double p2 = x * x + y * y;
    double p = std::sqrt(p2);
    double r = std::sqrt(p2 + z * z);
    double mu = std::atan((z / p) * ((1.0 - F) + E2 * A / r));
    double cos_mu = std::cos(mu);
    double sin_mu = std::sin(mu);
    double cos_mu_3 = cos_mu * cos_mu * cos_mu;
    double sin_mu_3 = sin_mu * sin_mu * sin_mu;
    double phi = std::atan((z * (1.0 - F) + E2 * A * sin_mu_3) / ((1.0 - F) * (p - E2 * A * cos_mu_3)));
    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);
    double sin_phi_2 = sin_phi * sin_phi;
    double h = p * cos_phi + z * sin_phi - A * std::sqrt(1.0 - E2 * sin_phi_2);
    double theta = std::atan2(y, x);
    return {phi, theta, h};
}

/**
 * @brief 楕円体座標系から直交座標系へ変換する。
 * @tparam T 楕円体座標系クラス
 * @param {b,l,h} 楕円体座標(緯度 [rad], 経度 [rad], 楕円体高 [m])
 * @return {x,y,z} 直交座標 [m]
 */
template<class T>
static inline std::tuple<double, double, double> blh2xyz(double phi, double theta, double h) {
    constexpr double A = T::A;
    constexpr double F = T::F;
    constexpr double E2 = 2.0 * F - F * F;
    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);
    double sin_phi_2 = sin_phi * sin_phi;
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double n = A / std::sqrt(1.0 - E2 * sin_phi_2);
    double x = (n + h) * cos_phi * cos_theta;
    double y = (n + h) * cos_phi * sin_theta;
    double z = (n * (1.0 - E2) + h) * sin_phi;
    return {x, y, z};
}

Wgs84::Wgs84(const Eigen::Vector3d& xyz) {
    auto [b, l, h] = xyz2blh<Wgs84>(xyz[0], xyz[1], xyz[2]);
    latitude = rad2deg(b);
    longitude = rad2deg(l);
    altitude = h;
}

Eigen::Vector3d Wgs84::xyz(void) const {
    auto [x, y, z] = blh2xyz<Wgs84>(deg2rad(latitude), deg2rad(longitude), altitude);
    return Eigen::Vector3d{x, y, z};
}

Eigen::Vector3d Wgs84::normal(void) const {
    double cos_lat = std::cos(deg2rad(latitude));
    double sin_lat = std::sin(deg2rad(latitude));
    double cos_lon = std::cos(deg2rad(longitude));
    double sin_lon = std::sin(deg2rad(longitude));
    return Eigen::Vector3d{
        cos_lat * cos_lon,
        cos_lat * sin_lon,
        sin_lat,
    };
}

} // namespace flat_earth
