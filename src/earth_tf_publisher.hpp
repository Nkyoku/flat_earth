/**
 * @file earth_tf_publisher.hpp
 * @author Fujii Naomichi (nkyoku@gmail.com)
 * @copyright Copyright (c) 2022 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "transform.hpp"
#include "wgs84.hpp"
#include <geographic_msgs/GeoPointStamped.h>
#include <memory>
#include <mutex>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace flat_earth {

class EarthTfPublisher : public nodelet::Nodelet {
public:
    EarthTfPublisher(void) {}

    virtual ~EarthTfPublisher() {}

    EarthTfPublisher(const EarthTfPublisher &) = delete;

private:
    virtual void onInit(void) override;

    void onTimer(const ros::TimerEvent &e);

    void onRequestReceived(const std_msgs::Time::ConstPtr &msg);

    void onReferenceReceived(const geographic_msgs::GeoPointStamped::ConstPtr &msg);

    void appendCurrentTransform(uint64_t stamp_ns, std::vector<geometry_msgs::TransformStamped> &tf_msgs);

    double getPositiveDoubleParameter(ros::NodeHandle &pnh, const char *name, double default_value);

    std::mutex _mutex;
    ros::Timer _timer;
    tf2_ros::Buffer _tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    ros::Duration _tf_timeout;
    ros::Subscriber _ref_subscriber;
    ros::Subscriber _request_subscriber;
    std::string _earth_frame_id = "earth";
    std::string _map_frame_id = "map";
    std::string _ref_frame_id = "ref";
    bool _allow_time_reversal = false;
    uint64_t _last_stamp_ns = 0;
    Eigen::Vector3d _ref_normal{0.0, 0.0, 0.0};
    Transform _e_r_tf;
    Transform _r_m_tf;
    Transform _e_m_tf;
};

} // namespace flat_earth
