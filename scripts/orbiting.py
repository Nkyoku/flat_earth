#!/usr/bin/python3

import math
import sys
import unittest
import numpy as np
from geographiclib.geodesic import Geodesic
import rospy
import tf2_ros as tf2
import tf2_geometry_msgs
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Vector3, Quaternion
from geographic_msgs.msg import GeoPoint, GeoPointStamped
import wgs84


def xyz_to_transform(pos: np.ndarray = np.zeros(3), theta: float = 0.0, *, stamp=rospy.Time(0)) -> TransformStamped:
    """map座標系上のgps座標を設定する"""
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.header.stamp = rospy.Time.now() if stamp.is_zero() else stamp
    transform.child_frame_id = "gps"
    transform.transform.translation = Vector3(pos[0], pos[1], pos[2])
    transform.transform.rotation = Quaternion(0.0, 0.0, math.sin(theta * 0.5), math.cos(theta * 0.5))
    return transform


def wgs84_to_geopoint(pos: wgs84.Wgs84, *, stamp=rospy.Time(0)) -> GeoPointStamped:
    """WGS84座標をGeoPointStampedに変換する。"""
    point = GeoPointStamped()
    point.header.frame_id = "gps"
    point.header.stamp = rospy.Time.now() if stamp.is_zero() else stamp
    point.position = GeoPoint(pos.latitude, pos.longitude, pos.altitude)
    return point


def heading_to_angle(heading: float, gps_frame_id: str, stamp: rospy.Time, tf_buffer: tf2.Buffer) -> float:
    """左手系北基準の方位角をmap座標系の角度に変換する。"""
    pole_point = PointStamped()
    pole_point.header.frame_id = "earth"
    pole_point.header.stamp = stamp
    pole_point.point = Point(0.0, 0.0, wgs84.B)
    pole_point = tf_buffer.transform(pole_point, "map", rospy.Duration(1))
    pole_pos = np.array((pole_point.point.x, pole_point.point.y))
    gps_tf: TransformStamped = tf_buffer.lookup_transform("map", gps_frame_id, stamp, rospy.Duration(1))
    gps_pos = np.array((gps_tf.transform.translation.x, gps_tf.transform.translation.y))
    direction = pole_pos - gps_pos
    length = np.linalg.norm(direction)
    if length < sys.float_info.epsilon:
        return 0.0
    return (math.atan2(direction[1], direction[0]) - heading) % (2 * math.pi)


class OrbitingTestCase(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def runTest(self):
        rospy.init_node("orbiting")
        ref_publisher = rospy.Publisher("ref", GeoPointStamped, queue_size=1, latch=True)

        tf_buffer = tf2.Buffer()
        tf_listener = tf2.TransformListener(tf_buffer)
        tf_broadcaster = tf2.TransformBroadcaster()

        # パラメータを取得する
        origin_lla = rospy.get_param("~origin", "").split()
        target_lla = rospy.get_param("~target", "").split()
        min_delta = rospy.get_param("~min_delta", 1.0)
        max_delta = rospy.get_param("~max_delta", 1000.0)
        tolerance = rospy.get_param("~tolerance", 1.0)

        # gps座標系の現在地
        origin_lla = [float(value) for value in origin_lla]
        origin_wgs84 = wgs84.Wgs84(*origin_lla)

        # gps座標系の目的地
        target_lla = [float(value) for value in target_lla]
        target_wgs84 = wgs84.Wgs84(*target_lla)
        target_point = target_wgs84.to_xyz()

        # 方位角と移動距離を決定する
        course = Geodesic.WGS84.Inverse(origin_wgs84.latitude, origin_wgs84.longitude, target_wgs84.latitude, target_wgs84.longitude)
        rospy.loginfo(f"Coodinate from {course['lat1']}, {course['lon1']} to {course['lat2']}, {course['lon2']}")
        rospy.loginfo(f"Initial heading is {course['azi1']} deg")
        rospy.loginfo(f"Geodesic distance is {course['s12']} m")
        origin_heading = course["azi1"] / 180.0 * math.pi
        distance = np.linalg.norm(origin_wgs84.to_xyz() - target_point)
        rospy.loginfo(f"Euclid distance is {distance} m")

        # 初期位置のWSG84座標とgps座標系へのTransformをpublishしつつ、map座標系へのTransformが利用可能になるまで待つ
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            tf_broadcaster.sendTransform(xyz_to_transform(stamp=stamp))
            ref_publisher.publish(wgs84_to_geopoint(origin_wgs84, stamp=stamp))
            if tf_buffer.can_transform("earth", "map", stamp, rospy.Duration(1)):
                break

        else:
            self.assertTrue(False, "TF timed out.")

        # map座標系でのgps座標系の初期姿勢
        gps_pos = np.zeros(3)
        gps_angle = heading_to_angle(origin_heading, "gps", stamp, tf_buffer)

        # 微小移動距離
        delta = max_delta

        # 合計移動距離
        total_movement = 0.0

        rospy.loginfo(f"Started.")

        # 目的地にたどり着くまで繰り返し移動する
        while not rospy.is_shutdown():
            last_stamp = stamp
            stamp = rospy.Time.now()

            # 目的地に到達したかどうか判定する
            if distance < min_delta:
                error = total_movement + distance - course['s12']
                rospy.loginfo(f"Goal!")
                rospy.loginfo(f"Total movement on /map frame is {total_movement} + {distance} m")
                rospy.loginfo(f"Error is {error} m")
                self.assertLessEqual(error, tolerance, f"Error distance exceeds tolerance. ({error} > {tolerance})")
                break

            # 移動距離を決定する
            delta = max(min(distance / 2, max_delta), min_delta)

            # map座標系上でgps座標系を移動し、Transformをpublishする
            gps_rotate = np.array([[math.cos(gps_angle), -math.sin(gps_angle), 0.0], [math.sin(gps_angle), math.cos(gps_angle), 0.0], [0.0, 0.0, 1.0]])
            gps_pos += np.matmul(gps_rotate, np.array([delta, 0.0, 0.0]))
            total_movement += delta
            tf_broadcaster.sendTransform(xyz_to_transform(gps_pos, gps_angle, stamp=stamp))

            # 移動した後のgps座標系の原点のGeoPointStampedをpublishする
            gps_point_msg = PointStamped(Header(0, last_stamp, "map"), Point(gps_pos[0], gps_pos[1], gps_pos[2]))
            try:
                gps_point_msg = tf_buffer.transform(gps_point_msg, "earth", rospy.Duration(1))
            except tf2.TransformException as e:
                self.assertTrue(False, e)
            gps_point = np.array([gps_point_msg.point.x, gps_point_msg.point.y, gps_point_msg.point.z])
            gps_wgs84 = wgs84.Wgs84.from_xyz(gps_point)
            gps_wgs84.altitude = gps_pos[2]  # 高度はmap座標系上のz座標とする
            ref_publisher.publish(wgs84_to_geopoint(gps_wgs84, stamp=stamp))

            last_distance = distance
            distance = np.linalg.norm(gps_point - target_point)
            self.assertLess(distance, last_distance, f"Distance increased to {distance} from {last_distance}")

        else:
            self.assertTrue(False, "Cancelled.")

        rospy.loginfo("Finished.")


if __name__ == '__main__':
    import rostest
    rostest.rosrun("flat_earth", 'orbiting', OrbitingTestCase)
