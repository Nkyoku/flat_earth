#pragma once

#include "transform.hpp"
#include "wgs84.hpp"
#include <geographic_msgs/GeoPointStamped.h>
#include <memory>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace flat_earth {

class EarthTfPublisher : public nodelet::Nodelet {
public:
    EarthTfPublisher(void) {}

    virtual ~EarthTfPublisher() {}

    EarthTfPublisher(const EarthTfPublisher &) = delete;

private:
    virtual void onInit(void) override;

    void onTimer(const ros::TimerEvent& e);

    void onOriginReceived(const geographic_msgs::GeoPointStamped::ConstPtr &origin);

    void appendCurrentTransform(uint64_t stamp_ns, std::vector<geometry_msgs::TransformStamped> &tf_msgs);

    ros::Timer _timer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    ros::Subscriber _origin_subscriber;
    std::string _earth_frame_id = "earth";
    std::string _map_frame_id = "map";
    uint64_t _last_stamp_ns = 0;
    uint32_t _stamp_offset_ns = 0;
    Eigen::Vector3d _contact_xyz{0.0, 0.0, 0.0};
    Eigen::Vector3d _contact_normal{0.0, 0.0, 0.0};
    Wgs84 _contact_wgs84;
    Transform _c2e_transform;
    Transform _o2c_transform;
};

} // namespace flat_earth
