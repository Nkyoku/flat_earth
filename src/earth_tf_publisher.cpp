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
    pnh.getParam("earth_frame_id", _earth_frame_id);
    pnh.getParam("map_frame_id", _map_frame_id);
    pnh.getParam("ref_frame_id", _ref_frame_id);
    pnh.getParam("allow_time_reversal", _allow_time_reversal);
    double tf_timeout = getPositiveDoubleParameter(pnh, "tf_timeout", 1.0);
    double publish_period = getPositiveDoubleParameter(pnh, "publish_period", 0.0);
    _tf_timeout = ros::Duration(tf_timeout);

    // Publisher, Subscriberを作成する
    constexpr int QUEUE_SIZE = 10;
    _tf_listener = std::make_unique<tf2_ros::TransformListener>(_tf_buffer);
    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
    _ref_subscriber = nh.subscribe("ref", QUEUE_SIZE, &EarthTfPublisher::onReferenceReceived, this);
    _request_subscriber = nh.subscribe("request", QUEUE_SIZE, &EarthTfPublisher::onRequestReceived, this);

    // 定期送信用のタイマーを作成する
    if (0.0 < publish_period) {
        _timer = nh.createTimer(ros::Duration(publish_period), &EarthTfPublisher::onTimer, this, false, false);
    }
}

void EarthTfPublisher::onTimer(const ros::TimerEvent& e) {
    auto msg = boost::make_shared<std_msgs::Time>();
    msg->data = ros::Time::now();
    onRequestReceived(msg);
}

void EarthTfPublisher::onRequestReceived(const std_msgs::Time::ConstPtr& msg) {
    std::unique_lock<std::mutex> lock(_mutex);

    uint64_t stamp_ns = msg->data.toNSec();
    if (!_allow_time_reversal && (stamp_ns <= _last_stamp_ns)) {
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

void EarthTfPublisher::onReferenceReceived(const geographic_msgs::GeoPointStamped::ConstPtr& msg) {
    std::unique_lock<std::mutex> lock(_mutex);

    // 基準座標のタイムスタンプを取得する。
    // タイムスタンプがないなら現在のROS時刻を使用する。
    uint64_t stamp_ns = msg->header.stamp.toNSec();
    if (stamp_ns == 0) {
        stamp_ns = ros::Time::now().toNSec();
    }

    if (!_allow_time_reversal && (stamp_ns <= _last_stamp_ns)) {
        // 最後に送信したタイムスタンプが今から送信しようとするタイムスタンプより未来なら送信を行わない
        return;
    }

    // GPS座標系からmap座標系への変換を取得する
    Transform m_g_tf;
    try {
        m_g_tf = _tf_buffer.lookupTransform(_map_frame_id, msg->header.frame_id, msg->header.stamp, _tf_timeout).transform;
    }
    catch (tf2::TransformException e) {
        NODELET_WARN("Cannot look up transform between '%s' and '%s'.", _map_frame_id.c_str(), msg->header.frame_id.c_str());
        return;
    }

    // 関数の最後にbroadcastするTFフレームはここに格納される
    auto tf_msgs = std::vector<geometry_msgs::TransformStamped>();

    // earth座標系におけるGPS座標とref座標系の原点を求める
    auto new_gps_wgs = Wgs84(msg->position.latitude, msg->position.longitude, msg->position.altitude);
    auto new_ref_wgs = Wgs84(msg->position.latitude, msg->position.longitude, 0.0);
    auto new_ref_xyz = new_ref_wgs.xyz();
    auto new_ref_normal = new_ref_wgs.normal();

    // 以下のブロックで求める変換
    Transform new_e_r_tf; // ref座標系からearth座標系への変換
    Transform new_r_m_tf; // map座標系からref座標系への変換
    Transform new_e_m_tf; // map座標系からearth座標系への変換

    if (_last_stamp_ns == 0) {
        // 起動して初めてのGPS座標を受け取った

        // ref座標系からearth座標系への変換を求める
        auto new_e_r_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d{0.0, 0.0, 1.0}, new_ref_normal);
        new_e_r_tf = Transform(new_e_r_quat, new_ref_xyz);

        // map座標系からearth座標系への変換を求める
        auto new_r_g_tf = Transform(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, -new_gps_wgs.altitude));
        new_r_m_tf = Transform::compose(new_r_g_tf, m_g_tf.inv());
        new_e_m_tf = Transform::compose(new_e_r_tf, new_r_m_tf);

        // 定期送信用のタイマーを開始する
        if (_timer.isValid()) {
            _timer.start();
        }
    }
    else {
        // 前回のGPS座標からの変化を反映する

        // 新しくref座標系からearth座標系への変換を求める
        auto new_r_old_r_quat = Eigen::Quaterniond::FromTwoVectors(_ref_normal, new_ref_normal);
        new_e_r_tf = Transform(new_r_old_r_quat * _e_r_tf.q, new_ref_xyz);

        // 古いref座標系から新しいref座標系への変換を求める
        auto new_r_old_r_tf = Transform::compose(new_e_r_tf.inv(), _e_r_tf);

        // 新しいmap座標系からref座標系への変換を求める
        // 回転成分をZ軸のみにする
        new_r_m_tf = Transform::compose(new_r_old_r_tf, _r_m_tf);
        new_r_m_tf.t[2] = new_gps_wgs.altitude - m_g_tf.t[2];
        new_r_m_tf.q.x() = 0.0;
        new_r_m_tf.q.y() = 0.0;
        new_r_m_tf.normalize();

        // 新しいmap座標系からearth座標系への変換を求める
        // GPS座標系の原点が実際のGPS座標と一致するようにmap座標系の原点を並行移動する
        new_e_m_tf = Transform::compose(new_e_r_tf, new_r_m_tf);
        auto new_e_g_tf = Transform::compose(new_e_m_tf, m_g_tf);
        new_e_m_tf.t += new_gps_wgs.xyz() - new_e_g_tf.t;
    }

    // 求めた変換を格納する
    _e_r_tf = new_e_r_tf;
    _r_m_tf = new_r_m_tf;
    _e_m_tf = new_e_m_tf;
    _ref_normal = new_ref_normal;

    // 現在のtfフレームを作成する
    appendCurrentTransform(stamp_ns, tf_msgs);

    // tfフレームを送信する
    if (!tf_msgs.empty()) {
        _tf_broadcaster->sendTransform(tf_msgs);
    }

    _last_stamp_ns = stamp_ns;
}

void EarthTfPublisher::appendCurrentTransform(uint64_t stamp_ns, std::vector<geometry_msgs::TransformStamped>& tf_msgs) {
    ros::Time stamp;
    stamp.fromNSec(stamp_ns);
    tf_msgs.push_back(_e_m_tf.toMsg(_earth_frame_id, _map_frame_id, stamp));
    if (!_ref_frame_id.empty()) {
        tf_msgs.push_back(_e_r_tf.toMsg(_earth_frame_id, _ref_frame_id, stamp));
    }
}

double EarthTfPublisher::getPositiveDoubleParameter(ros::NodeHandle& pnh, const char* name, double default_value) {
    double value = default_value;
    pnh.getParam(name, value);
    if (!(0.0 <= value)) {
        NODELET_ERROR("Parameter '%s' must be positive or zero, but the value is %f.", name, value);
        throw std::exception();
    }
    return value;
}

} // namespace flat_earth

PLUGINLIB_EXPORT_CLASS(flat_earth::EarthTfPublisher, nodelet::Nodelet);
