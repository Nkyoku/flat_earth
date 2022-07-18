#!/usr/bin/python3

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import wgs84
from plot import Wgs84EllipsoidPlot, Wgs84PathPlot, PathPlot2d, AxesPlot3d
import rospy
import tf2_ros as tf2
import tf2_geometry_msgs
from std_msgs.msg import Header, Time
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from geographiclib.geodesic import Geodesic


def plot_axis_2d(ax: plt.Axes, length: float, transform: np.ndarray):
    pt0 = transform[0:3, 3]
    pt1 = np.matmul(transform[0:3, 0:3], np.array([length, 0, 0])) + transform[0:3, 3]
    pt2 = np.matmul(transform[0:3, 0:3], np.array([0, length, 0])) + transform[0:3, 3]
    ax.plot([pt0[0], pt1[0]], [pt0[1], pt1[1]], color="r")
    ax.plot([pt0[0], pt2[0]], [pt0[1], pt2[1]], color="g")


def plot_axis_3d(ax: plt.Axes, length: float, transform: np.ndarray):
    pt0 = transform[0:3, 3]
    pt1 = np.matmul(transform[0:3, 0:3], np.array([length, 0, 0])) + transform[0:3, 3]
    pt2 = np.matmul(transform[0:3, 0:3], np.array([0, length, 0])) + transform[0:3, 3]
    pt3 = np.matmul(transform[0:3, 0:3], np.array([0, 0, length])) + transform[0:3, 3]
    ax.plot([pt0[0], pt1[0]], [pt0[1], pt1[1]], color="r")
    ax.plot([pt0[0], pt2[0]], [pt0[1], pt2[1]], color="g")
    ax.plot([pt0[0], pt3[0]], [pt0[1], pt3[1]], color="b")


def wgs84_to_geopoint(pos: wgs84.Wgs84, *, stamp=rospy.Time(0)) -> GeoPointStamped:
    origin = GeoPointStamped()
    origin.header.frame_id = "gps"
    origin.header.stamp = rospy.Time.now() if stamp.is_zero() else stamp
    origin.position.latitude = pos.latitude
    origin.position.longitude = pos.longitude
    origin.position.altitude = pos.altitude
    return origin


def xyz_to_transform(pos: np.ndarray = np.zeros(3), theta: float = 0.0, *, stamp=rospy.Time(0)) -> TransformStamped:
    m_g_tf = TransformStamped()
    m_g_tf.header.frame_id = "map"
    m_g_tf.header.stamp = rospy.Time.now() if stamp.is_zero() else stamp
    m_g_tf.child_frame_id = "gps"
    m_g_tf.transform.translation.x = pos[0]
    m_g_tf.transform.translation.y = pos[1]
    m_g_tf.transform.translation.z = pos[2]
    m_g_tf.transform.rotation.z = math.sin(theta * 0.5)
    m_g_tf.transform.rotation.w = math.cos(theta * 0.5)
    return m_g_tf


def get_north_pole(stamp: rospy.Time, tf_buffer: tf2.Buffer) -> np.ndarray:
    pole_point = PointStamped()
    pole_point.header.frame_id = "earth"
    pole_point.header.stamp = stamp
    pole_point.point.x = 0.0
    pole_point.point.y = 0.0
    pole_point.point.z = wgs84.B
    pole_point = tf_buffer.transform(pole_point, "map", rospy.Duration(1))
    return np.array((pole_point.point.x, pole_point.point.y))


def heading_to_angle(heading: float, gps_frame_id: str, stamp: rospy.Time, tf_buffer: tf2.Buffer) -> float:
    """左手系北基準の方位角をmap座標系の角度に変換する。"""
    pole_pos = get_north_pole(stamp, tf_buffer)
    gps_tf: TransformStamped = tf_buffer.lookup_transform("map", gps_frame_id, stamp, rospy.Duration(1))
    gps_pos = np.array((gps_tf.transform.translation.x, gps_tf.transform.translation.y))
    direction = pole_pos - gps_pos
    length = np.linalg.norm(direction)
    if length < sys.float_info.epsilon:
        return 0.0
    return (math.atan2(direction[1], direction[0]) - heading) % (2 * math.pi)


def angle_to_heading(gps_pos: np.ndarray, gps_angle: float, stamp: rospy.Time, tf_buffer: tf2.Buffer) -> float:
    pole_pos = get_north_pole(stamp, tf_buffer)
    a = np.array([math.cos(gps_angle), math.sin(gps_angle)])
    b = pole_pos - gps_pos[0:2]
    b /= np.linalg.norm(b)
    theta = math.acos(np.dot(a, b))
    if np.cross(a, b) < 0.0:
        theta = math.pi * 2 - theta
    return theta


if __name__ == "__main__":
    rospy.init_node("demo")
    ref_publisher = rospy.Publisher("ref", GeoPointStamped, queue_size=1, latch=True)
    request_publisher = rospy.Publisher("request", Time, queue_size=1, latch=True)

    tf_buffer = tf2.Buffer()
    tf_listener = tf2.TransformListener(tf_buffer)
    tf_broadcaster = tf2.TransformBroadcaster()

    # gps座標系の現在地
    origin_wgs84 = wgs84.Wgs84(35.0, 135.0, 0.0)

    # gps座標系の目的地
    destinaton_wgs84 = wgs84.Wgs84(55.0, 5.0, 0.0)

    # 方位角を決定する
    course = Geodesic.WGS84.Inverse(origin_wgs84.latitude, origin_wgs84.longitude, destinaton_wgs84.latitude, destinaton_wgs84.longitude)
    rospy.loginfo(f"Coodinate from {course['lat1']}, {course['lon1']} to {course['lat2']}, {course['lon2']}")
    rospy.loginfo(f"Initial heading is {course['azi1']}")
    origin_heading = course["azi1"] / 180.0 * math.pi

    # TFフレームを待つ
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()

        # map<-gpsへの変換をbroadcastする
        tf_broadcaster.sendTransform(xyz_to_transform(stamp=stamp))

        # 原点をパブリッシュする
        ref_publisher.publish(wgs84_to_geopoint(origin_wgs84, stamp=stamp))

        # 必要な変換の受信を待つ
        if tf_buffer.can_transform("earth", "map", stamp, rospy.Duration(1)):
            break
        rospy.loginfo("Waiting TF frame...")
    else:
        exit()

    # map座標系でのgps座標系の初期姿勢
    gps_pos = np.zeros(3)
    gps_angle = heading_to_angle(origin_heading, "gps", stamp, tf_buffer)

    # プロットを作成する
    plt.rcParams['keymap.save'].remove('s')
    plt.rcParams['keymap.quit'].remove('q')
    fig = plt.figure(constrained_layout=True)
    gs = fig.add_gridspec(ncols=2, nrows=2, width_ratios=[1, 1], height_ratios=[1, 1])
    ax1: plt.Axes = fig.add_subplot(gs[0:2, 0], projection='3d')
    ax2: plt.Axes = fig.add_subplot(gs[0, 1])
    ax3: plt.Axes = fig.add_subplot(gs[1, 1])

    # 地球楕円体の3Dプロットを表示する
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_zlabel("z")
    ax1.set_xlim((-wgs84.A, wgs84.A))
    ax1.set_ylim((-wgs84.A, wgs84.A))
    ax1.set_zlim((-wgs84.A, wgs84.A))
    ax1.set_box_aspect((1, 1, 1))

    # メルカトル図法で表示する
    ax2.set_xlabel("lon")
    ax2.set_ylabel("lat")
    ax2.set_xlim((-180, 180))
    ax2.set_ylim((-90, 90))
    ax2.set_xticks(np.linspace(-180, 180, 25))
    ax2.set_yticks(np.linspace(-90, 90, 13))
    ax2.grid()

    # プロットオブジェクトを作成する
    ellipsoid = Wgs84EllipsoidPlot()
    path_wgs84 = Wgs84PathPlot(color="r")
    path_2d = PathPlot2d(color="r")
    axes_3d = AxesPlot3d()
    axes_3d.scale = 1000000

    # 押下中のキーはpressing_keyに反映される
    pressing_key: "set[str]" = set()
    fig.canvas.mpl_connect('key_press_event', lambda event: [pressing_key.add(event.key)])
    fig.canvas.mpl_connect('key_release_event', lambda event: [pressing_key.remove(event.key)])
    plt.figtext(0.001, 0.001, "'W' : Move forward, 'S' : Move back\n'A' : Move left, 'D' : Move right\n'Z' : Move Down, 'X' : Move up\n'Q' : Turn left, 'E' : Turn right\n'Esc' : Exit demo")

    while not rospy.is_shutdown():
        last_stamp = stamp
        stamp = rospy.Time.now()

        # 終了キー
        if "escape" in pressing_key:
            break

        # 移動キー
        move_dx = 0.0
        move_dy = 0.0
        move_dz = 0.0
        move_dw = 0.0
        if "w" in pressing_key:
            move_dx += 100000
        if "a" in pressing_key:
            move_dy += 100000
        if "s" in pressing_key:
            move_dx -= 100000
        if "d" in pressing_key:
            move_dy -= 100000
        if "z" in pressing_key:
            move_dz -= 100000
        if "x" in pressing_key:
            move_dz += 100000
        if "e" in pressing_key:
            move_dw -= math.pi / 10
        if "q" in pressing_key:
            move_dw += math.pi / 10

        update_plot = False
        if (move_dx != 0.0) or (move_dy != 0.0) or (move_dz != 0.0) or (move_dw != 0.0):
            gps_angle += move_dw
            gps_pos += np.matmul(np.array([[math.cos(gps_angle), -math.sin(gps_angle), 0.0], [math.sin(gps_angle), math.cos(gps_angle), 0.0], [0.0, 0.0, 1.0]]),
                                 np.array([move_dx, move_dy, move_dz]))
            update_plot = True

        # GPS座標系への変換をpublishする
        tf_broadcaster.sendTransform(xyz_to_transform(gps_pos, gps_angle, stamp=stamp))

        if update_plot:
            # 移動した後のgps座標系の原点のGeoPointStampedをpublishする
            gps_point_msg = PointStamped(Header(0, last_stamp, "map"), Point(gps_pos[0], gps_pos[1], gps_pos[2]))
            try:
                gps_point_msg = tf_buffer.transform(gps_point_msg, "earth", rospy.Duration(1))
            except tf2.TransformException as e:
                rospy.logerr(e)
            gps_point = np.array([gps_point_msg.point.x, gps_point_msg.point.y, gps_point_msg.point.z])
            gps_wgs84 = wgs84.Wgs84.from_xyz(gps_point)
            gps_wgs84.altitude = gps_pos[2]  # 高度はmap座標系上のz座標とする
            ref_publisher.publish(wgs84_to_geopoint(gps_wgs84, stamp=stamp))

            # プロットデータを更新する
            path_2d.append_xyz(gps_pos)
            path_wgs84.append_wgs84(gps_wgs84)
            ax3.relim()
            ax3.autoscale_view()

            # 座標と方位を出力する
            heading = angle_to_heading(gps_pos, gps_angle, stamp, tf_buffer) / math.pi * 180.0
            rospy.loginfo(
                f"XYZ=[{gps_pos[0]:.0f}, {gps_pos[1]:.0f}, {gps_pos[2]:.0f}], LLA=[{gps_wgs84.latitude:16.12f}, {gps_wgs84.longitude:17.12f}, {gps_wgs84.altitude:.3f}], Heading={heading}")
        else:
            # flat_earthノードにTFだけpublishさせる
            request_publisher.publish(Time(stamp))

        # GPS座標系を描画する
        axes_3d.set_transform("gps", "earth", stamp, tf_buffer)

        # プロットを更新する
        ellipsoid.plot(ax1)
        path_wgs84.plot3d(ax1)
        path_wgs84.plot2d(ax2)
        path_2d.plot2d(ax3)
        axes_3d.plot3d(ax1)

        plt.pause(0.1)

    rospy.loginfo("Shutting down...")
