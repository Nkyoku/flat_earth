/**
 * @file earth_tf_publisher.cpp
 * @author Fujii Naomichi (nkyoku@gmail.com)
 * @copyright Copyright (c) 2022 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#include "earth_tf_publisher.hpp"
#include "geometry.hpp"
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <stdexcept>

namespace flat_earth {

void EarthTfPublisher::onInit(void) {
    std::unique_lock<std::mutex> lock(_mutex);

    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle pnh = getMTPrivateNodeHandle();

    // パラメータを取得する
    double publish_period = 0.0;
    double stamp_offset = 0.0;
    pnh.getParam("earth_frame_id", _earth_frame_id);
    pnh.getParam("map_frame_id", _map_frame_id);
    pnh.getParam("ref_frame_id", _ref_frame_id);
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
    _ref_subscriber = nh.subscribe("ref", QUEUE_SIZE, &EarthTfPublisher::onReferenceReceived, this);

    // 定期送信用のタイマーを作成する
    if (0.0 < publish_period) {
        _timer = nh.createTimer(ros::Duration(publish_period), &EarthTfPublisher::onTimer, this, false, false);
    }
}

void EarthTfPublisher::onTimer(const ros::TimerEvent& e) {
    std::unique_lock<std::mutex> lock(_mutex);

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

void EarthTfPublisher::onReferenceReceived(const geographic_msgs::GeoPointStamped::ConstPtr& ref) {
    std::unique_lock<std::mutex> lock(_mutex);

    // 基準座標のタイムスタンプを取得する。
    // タイムスタンプがないなら現在のROS時刻を使用する。
    uint64_t stamp_ns = ref->header.stamp.toNSec();
    if (stamp_ns == 0) {
        stamp_ns = ros::Time::now().toNSec();
    }
    stamp_ns += _stamp_offset_ns;

    auto tf_msgs = std::vector<geometry_msgs::TransformStamped>();

    // 地球中心座標系における基準座標での接平面の法線ベクトルを求める
    auto new_ref_wgs84 = Wgs84(ref->position.latitude, ref->position.longitude, ref->position.altitude);
    auto new_ref_xyz = new_ref_wgs84.xyz();
    auto new_ref_normal = new_ref_wgs84.normal();

    if (_last_stamp_ns == 0) {
        // 起動して初めての原点を受け取った

        // 基準座標系から地球中心座標系への変換行列を求める
        auto new_r2e_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d{0.0, 0.0, 1.0}, new_ref_normal);
        _r2e_transform = Transform(new_r2e_quat, new_ref_xyz);
        _m2e_transform = _r2e_transform;

        // map座標系から基準座標系への変換行列を設定する
        _m2r_transform = {};

        // 本来の現在時刻のtfフレームを作成する
        if (0 < _stamp_offset_ns) {
            appendCurrentTransform(stamp_ns - _stamp_offset_ns, tf_msgs);
        }

        // 定期送信用のタイマーを開始する
        if (_timer.isValid()) {
            _timer.start();
        }
    }
    else {
        // 新しい基準座標系から地球中心座標系への変換行列を求める(方法1)
        // Pros. 南極点において特異点が発生しない
        // Cons. 基準座標系の変換行列が一意に定まらない
        auto old2new_r2r_quat = Eigen::Quaterniond::FromTwoVectors(_ref_normal, new_ref_normal);
        auto new_r2e_transform = Transform(old2new_r2r_quat * _r2e_transform.q, new_ref_xyz);

        // 新しい基準座標系から地球中心座標系への変換行列を求める(方法2)
        // Pros. Lat,Lonが分かれば基準座標系の変換行列が一意に定まる
        // Cons. 南極点において特異点が発生する
        // auto new_r2e_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d{0.0, 0.0, 1.0}, new_ref_normal);
        // auto new_r2e_transform = Transform(new_r2e_quat, new_ref_xyz);

        // 古い基準座標系から新しい基準座標系への変換行列を求める
        auto old2new_r2r_transform = Transform::compose(new_r2e_transform.inv(), _r2e_transform);

        // 新しいmap座標系から基準座標系への変換行列を求める
        // 2次元化する
        auto new_m2r_transform = Transform::compose(old2new_r2r_transform, _m2r_transform);
        new_m2r_transform.q.x() = 0.0;
        new_m2r_transform.q.y() = 0.0;
        new_m2r_transform.t[2] = 0.0;
        new_m2r_transform.normalize();

        // 値を更新する
        _r2e_transform = new_r2e_transform;
        _m2r_transform = new_m2r_transform;
        _m2e_transform = Transform::compose(_r2e_transform, _m2r_transform);
        _ref_xyz = new_ref_xyz;
        _ref_wgs84 = new_ref_wgs84;
        _ref_normal = new_ref_normal;
    }

    // 現在のtfフレームを作成する
    if (_last_stamp_ns < stamp_ns) {
        appendCurrentTransform(stamp_ns, tf_msgs);
    }

    // tfフレームを送信する
    if (!tf_msgs.empty()) {
        _tf_broadcaster->sendTransform(tf_msgs);
    }

    _last_stamp_ns = stamp_ns;
}

void EarthTfPublisher::appendCurrentTransform(uint64_t stamp_ns, std::vector<geometry_msgs::TransformStamped>& tf_msgs) {
    ros::Time stamp;
    stamp.fromNSec(stamp_ns);
    tf_msgs.push_back(_m2e_transform.toMsg(_earth_frame_id, _map_frame_id, stamp));
    if (!_ref_frame_id.empty()) {
        tf_msgs.push_back(_r2e_transform.toMsg(_earth_frame_id, _ref_frame_id, stamp));
    }
}

} // namespace flat_earth

PLUGINLIB_EXPORT_CLASS(flat_earth::EarthTfPublisher, nodelet::Nodelet);
