#include "earth_tf_publisher.hpp"
#include "geometry.hpp"
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <stdexcept>

namespace flat_earth {

void EarthTfPublisher::onInit(void) {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();

    // パラメータを取得する
    double publish_period = 0.5;
    double stamp_offset = 1.0;
    pnh.getParam("earth_frame_id", _earth_frame_id);
    pnh.getParam("map_frame_id", _map_frame_id);
    pnh.getParam("publish_period", publish_period);
    pnh.getParam("stamp_offset", stamp_offset);
    if (!std::isfinite(stamp_offset) || (stamp_offset < 0.0)) {
        NODELET_ERROR("Incorrect parameter 'stamp_offset', the value is %f", stamp_offset);
        throw std::exception();
    }
    _stamp_offset_ns = (uint64_t)(stamp_offset * 1000000000);

    // Publisher, Subscriberを作成する
    constexpr int QUEUE_SIZE = 10;
    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
    _origin_subscriber = nh.subscribe("origin", QUEUE_SIZE, &EarthTfPublisher::onOriginReceived, this);

    // 定期送信用のタイマーを作成する
    if (0.0 < publish_period) {
        _timer = nh.createTimer(ros::Duration(publish_period), &EarthTfPublisher::onTimer, this);
    }
}

void EarthTfPublisher::onTimer(const ros::TimerEvent& e) {
    if (_last_stamp_ns == 0) {
        // tfフレームを送信したことがないなら定期送信を行わない
        return;
    }

    uint64_t stamp_ns = ros::Time::now().toNSec() + _stamp_offset_ns;
    if (stamp_ns <= _last_stamp_ns) {
        // 最後に送信したタイムスタンプが今から送信しようとするタイムスタンプより未来なら定期送信を行わない
        return;
    }

    // tfフレームを作成する
    auto tf_msgs = std::vector<geometry_msgs::TransformStamped>();
    appendCurrentTransform(stamp_ns, tf_msgs);

    // tfフレームを送信する
    _tf_broadcaster->sendTransform(tf_msgs);

    _last_stamp_ns = stamp_ns;
}

void EarthTfPublisher::onOriginReceived(const geographic_msgs::GeoPointStamped::ConstPtr& origin) {
    uint64_t stamp_ns = ros::Time::now().toNSec() + _stamp_offset_ns;

    auto tf_msgs = std::vector<geometry_msgs::TransformStamped>();

    // 地球中心座標系における接点での接平面の法線ベクトルを求める
    auto new_contact_wgs84 = Wgs84(origin->position.latitude, origin->position.longitude, origin->position.altitude);
    auto new_contact_xyz = new_contact_wgs84.xyz();
    auto new_contact_normal = new_contact_wgs84.normal();

    if (_last_stamp_ns == 0) {
        // 起動して初めての原点を受け取った

        // 接点座標系から地球中心座標系への変換行列を求める
        auto new_c2e_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d{0.0, 0.0, 1.0}, new_contact_normal);
        _c2e_transform = Transform(new_c2e_quat, new_contact_xyz);

        // 基準座標系から接点座標系への変換行列を設定する
        _o2c_transform = {};

        if (0 < _stamp_offset_ns) {
            // 本来の現在時刻のtfフレームを作成する
            appendCurrentTransform(stamp_ns - _stamp_offset_ns, tf_msgs);
        }
    }
    else {
        // 新しい接点座標系から地球中心座標系への変換行列を求める(方法1)
        // Pros. 南極点において特異点が発生しない
        // Cons. 接点座標系の変換行列が一意に定まらない
        auto old2new_c2c_quat = Eigen::Quaterniond::FromTwoVectors(_contact_normal, new_contact_normal);
        auto new_c2e_transform = Transform(old2new_c2c_quat * _c2e_transform.q, new_contact_xyz);

        // 新しい接点座標系から地球中心座標系への変換行列を求める(方法2)
        // Pros. Lat,Lonが分かれば接点座標系の変換行列が一意に定まる
        // Cons. 南極点において特異点が発生する
        // auto new_c2e_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d{0.0, 0.0, 1.0}, new_contact_normal);
        // auto new_c2e_transform = Transform(new_c2e_quat, new_contact_xyz);

        // 古い接点座標系から新しい接点座標系への変換行列を求める
        auto old2new_c2c_transform = Transform::compose(new_c2e_transform.inv(), _c2e_transform);

        // 新しい基準座標系から接点座標系への変換行列を求める
        // 2次元化する
        auto new_o2c_transform = Transform::compose(old2new_c2c_transform, _o2c_transform);
        new_o2c_transform.q.x() = 0.0;
        new_o2c_transform.q.y() = 0.0;
        new_o2c_transform.t[2] = 0.0;
        new_o2c_transform.normalize();

        // 値を更新する
        _c2e_transform = new_c2e_transform;
        _o2c_transform = new_o2c_transform;
        _contact_xyz = new_contact_xyz;
        _contact_wgs84 = new_contact_wgs84;
        _contact_normal = new_contact_normal;
    }

    // 現在のtfフレームを作成する
    appendCurrentTransform(stamp_ns, tf_msgs);

    // tfフレームを送信する
    _tf_broadcaster->sendTransform(tf_msgs);

    _last_stamp_ns = stamp_ns;
}

void EarthTfPublisher::appendCurrentTransform(uint64_t stamp_ns, std::vector<geometry_msgs::TransformStamped>& tf_msgs) {
    ros::Time stamp;
    stamp.fromNSec(stamp_ns);
    tf_msgs.push_back(_o2c_transform.toMsg("contact", _map_frame_id, stamp));
    tf_msgs.push_back(_c2e_transform.toMsg(_earth_frame_id, "contact", stamp));
}

} // namespace flat_earth

PLUGINLIB_EXPORT_CLASS(flat_earth::EarthTfPublisher, nodelet::Nodelet);
