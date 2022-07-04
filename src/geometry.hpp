#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <limits>

namespace flat_earth {

static Eigen::Vector3d rotvec(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    // a, bは正規化済みとする
    Eigen::Vector3d c = a.cross(b);
    double n = c.norm();
    if (std::numeric_limits<double>::epsilon() < n) {
        c *= (std::acos(a.dot(b)) / n);
    }
    return c;
}

} // namespace flat_earth
