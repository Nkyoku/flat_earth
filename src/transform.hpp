#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

namespace flat_earth {

class Transform {
public:
    Eigen::Quaterniond q;
    Eigen::Vector3d t;

    Transform(void) : q(1.0, 0.0, 0.0, 0.0), t(0.0, 0.0, 0.0) {}

    Transform(const Eigen::Quaterniond &q, const Eigen::Vector3d &t) : q(q), t(t) {
        normalize();
    }

    Transform(const geometry_msgs::Transform &msg)
        : q(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z), t(msg.translation.x, msg.translation.y, msg.translation.z) {
        normalize();
    }

    geometry_msgs::Transform toMsg(void) const {
        geometry_msgs::Transform msg;
        msg.rotation.w = q.w();
        msg.rotation.x = q.x();
        msg.rotation.y = q.y();
        msg.rotation.z = q.z();
        msg.translation.x = t[0];
        msg.translation.y = t[1];
        msg.translation.z = t[2];
        return msg;
    }

    geometry_msgs::TransformStamped toMsg(const std::string &base_frame_id, const std::string &child_frame_id, ros::Time stamp, uint32_t seq = 0) const {
        geometry_msgs::TransformStamped msg;
        msg.header.seq = seq;
        msg.header.stamp = stamp;
        msg.header.frame_id = base_frame_id;
        msg.child_frame_id = child_frame_id;
        msg.transform = toMsg();
        return msg;
    }


    Eigen::Matrix4d mat44(void) const {
        Eigen::Matrix4d result;
        result.block<3, 3>(0, 0) = q.matrix();
        result.block<3, 1>(0, 3) = t;
        result.block<1, 3>(3, 0) = Eigen::RowVector3d{0.0, 0.0, 0.0};
        result(3, 3) = 1.0;
        return result;
    }

    void normalize(void) {
        q.normalize();
    }

    Transform inv(void) const {
        Eigen::Quaterniond inv_q = q.inverse();
        Eigen::Vector3d inv_t = -inv_q.matrix() * t;
        return Transform(inv_q, inv_t);
    }

    Eigen::Vector3d apply(const Eigen::Vector3d &v) const {
        return q.matrix() * v + t;
    }

    static Transform compose(const Transform &a, const Transform &b) {
        Eigen::Matrix4d c = a.mat44() * b.mat44();
        return Transform(Eigen::Quaterniond(c.block<3, 3>(0, 0)), Eigen::Vector3d(c.block<3, 1>(0, 3)));
    }
};

} // namespace flat_earth
