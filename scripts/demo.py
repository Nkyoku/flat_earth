#!/usr/bin/python3

import queue
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import math
import wgs84
from plot import Wgs84EllipsoidPlot, Wgs84PathPlot, PathPlot2d
import copy
import rospy
import tf2_ros
import tf2_geometry_msgs
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PointStamped, PoseStamped


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


def wgs84_to_geopoint(pos: wgs84.Wgs84) -> GeoPointStamped:
    origin = GeoPointStamped()
    origin.header.frame_id = ""
    origin.header.stamp = rospy.Time.now()
    origin.position.latitude = pos.latitude
    origin.position.longitude = pos.longitude
    origin.position.altitude = pos.altitude
    return origin


if __name__ == "__main__":
    rospy.init_node("hamster")
    origin_publisher = rospy.Publisher("origin", GeoPointStamped, queue_size=1, latch=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1.0)

    # 原点をパブリッシュする
    origin_publisher.publish(wgs84_to_geopoint(wgs84.Wgs84(0.0, 0.0, 0.0)))

    # TFフレームを待つ
    while True:
        if tf_buffer.can_transform("earth", "map", rospy.Time(0), rospy.Duration(1)):
            if tf_buffer.can_transform("earth", "contact", rospy.Time(0), rospy.Duration(1)):
                break
        if rospy.is_shutdown():
            exit()
        rospy.loginfo("Waiting TF frame...")

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

    # map座標系を基準として表示する
    # ax3

    # プロットオブジェクトを作成する
    ellipsoid = Wgs84EllipsoidPlot()
    path_wgs84 = Wgs84PathPlot(color="r")
    path_2d = PathPlot2d(color="r")
    plane_3d = None

    # 押下中のキーはpressing_keyに反映される
    pressing_key: "set[str]" = set()
    fig.canvas.mpl_connect('key_press_event', lambda event: [pressing_key.add(event.key)])
    fig.canvas.mpl_connect('key_release_event', lambda event: [pressing_key.remove(event.key)])

    move_xyz = np.zeros(3)
    move_theta = 0.0

    while not rospy.is_shutdown():
        # 終了キー
        if "escape" in pressing_key:
            break

        # 移動キー
        move_dx = 0.0
        move_dy = 0.0
        if "w" in pressing_key:
            move_dy += 100
        if "a" in pressing_key:
            move_dx -= 100
        if "s" in pressing_key:
            move_dy -= 100
        if "d" in pressing_key:
            move_dx += 100
        if "e" in pressing_key:
            move_theta -= math.pi / 10
        if "q" in pressing_key:
            move_theta += math.pi / 10
        if (move_dx != 0.0) or (move_dy != 0.0):
            move_xyz += np.matmul(np.array([[math.cos(move_theta), -math.sin(move_theta), 0.0], [math.sin(move_theta), math.cos(move_theta), 0.0], [0.0, 0.0, 1.0]]),
                                  np.array([move_dx, move_dy, 0.0]))
            path_2d.append_xyz(move_xyz)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = move_xyz[0]
            pose.pose.position.y = move_xyz[1]
            pose.pose.position.z = move_xyz[2]
            pose.pose.orientation.w = 1.0
            for _ in range(1):
                pose = tf_buffer.transform(pose, "earth")

            current_position = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

            current_wgs84 = wgs84.Wgs84.from_xyz(current_position)
            current_wgs84.altitude = 0.0
            origin_publisher.publish(wgs84_to_geopoint(current_wgs84))

            #current_wgs84.altitude = move_xyz[2]
            #current_position = current_wgs84.to_xyz()

            path_wgs84.append_wgs84(current_wgs84)

            ax3.relim()
            ax3.autoscale_view()

            pass
        
        

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = move_xyz[0]
        pose.pose.position.y = move_xyz[1]
        pose.pose.position.z = move_xyz[2]
        pose.pose.orientation.w = 1.0
        for _ in range(1):
            pose = tf_buffer.transform(pose, "earth")
        current_position = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        print(f"Time:{rospy.Time.now()}, pos={current_position}, {'*' if (move_dx != 0.0) or (move_dy != 0.0) else ''}")



        # 接平面を描画する
        plane_data = np.array([[-1000000, -1000000, 0], [-1000000, 1000000, 0], [1000000, 1000000, 0],
                               [1000000, -1000000, 0], [-1000000, -1000000, 0], [0, 0, 0], [0, 0, 1000000]])
        for row in range(plane_data.shape[0]):
            pose = PoseStamped()
            pose.header.frame_id = "contact"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = plane_data[row, 0]
            pose.pose.position.y = plane_data[row, 1]
            pose.pose.position.z = plane_data[row, 2]
            pose.pose.orientation.w = 1.0
            for _ in range(1):
                pose = tf_buffer.transform(pose, "earth")
            plane_data[row, 0] = pose.pose.position.x
            plane_data[row, 1] = pose.pose.position.y
            plane_data[row, 2] = pose.pose.position.z
        if plane_3d is None:
            plane_3d = ax1.plot(plane_data[:, 0], plane_data[:, 1], plane_data[:, 2], color="g")[0]
        else:
            plane_3d.set_data_3d(plane_data[:, 0], plane_data[:, 1], plane_data[:, 2])

        ellipsoid.plot(ax1)
        path_wgs84.plot3d(ax1)
        path_wgs84.plot2d(ax2)
        path_2d.plot2d(ax3)

        plt.pause(0.1)

    rospy.loginfo("Shutting down...")
