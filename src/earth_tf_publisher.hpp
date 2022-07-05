#pragma once

#include "transform.hpp"
#include "wgs84.hpp"
#include <geographic_msgs/GeoPointStamped.h>
#include <memory>
#include <mutex>
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

    void onTimer(const ros::TimerEvent &e);

    void onReferenceReceived(const geographic_msgs::GeoPointStamped::ConstPtr &ref);

    void appendCurrentTransform(uint64_t stamp_ns, std::vector<geometry_msgs::TransformStamped> &tf_msgs);

    std::mutex _mutex;
    ros::Timer _timer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    ros::Subscriber _ref_subscriber;
    std::string _earth_frame_id = "earth";
    std::string _map_frame_id = "map";
    std::string _ref_frame_id = "ref";
    uint64_t _last_stamp_ns = 0;
    uint64_t _stamp_offset_ns = 0;
    Eigen::Vector3d _ref_xyz{0.0, 0.0, 0.0};
    Eigen::Vector3d _ref_normal{0.0, 0.0, 0.0};
    Wgs84 _ref_wgs84;
    Transform _r2e_transform;
    Transform _m2r_transform;
    Transform _m2e_transform;
};

} // namespace flat_earth
