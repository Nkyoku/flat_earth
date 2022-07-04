#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

namespace flat_earth {

static inline geometry_msgs::Transform toMsg(const Eigen::Quaterniond &q, const Eigen::Vector3d &t) {
    geometry_msgs::Transform transform;
    transform.rotation = tf2::toMsg(q);
    tf2::toMsg(t, transform.translation);
    return transform;
}

static inline geometry_msgs::TransformStamped toMsg(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const std::string &parent_frame_id,
                                                    const std::string &child_frame_id, const ros::Time &stamp) {
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = parent_frame_id;
    transform.header.stamp = stamp;
    transform.child_frame_id = child_frame_id;
    transform.transform.rotation = tf2::toMsg(q);
    tf2::toMsg(t, transform.transform.translation);
    return transform;
}

static inline Eigen::Quaterniond fromMsg(const geometry_msgs::Quaternion &q) {
    return Eigen::Quaterniond{q.w, q.x, q.y, q.z};
}

static inline Eigen::Vector3d fromMsg(const geometry_msgs::Vector3 &t) {
    return Eigen::Vector3d{t.x, t.y, t.z};
}

} // namespace flat_earth
