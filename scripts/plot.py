#!/usr/bin/python3

from typing import Iterable
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from mpl_toolkits import mplot3d
import numpy as np
import wgs84


class Wgs84EllipsoidPlot():
    def __init__(self, **plot_kwargs):
        self.__plot_kwargs = plot_kwargs
        self.__obj = None

        # 楕円体の座標データを生成する
        u = np.linspace(0, 2 * np.pi, 13)
        v = np.linspace(0, np.pi, 13)
        self.__x = wgs84.A * np.outer(np.cos(u), np.sin(v))
        self.__y = wgs84.A * np.outer(np.sin(u), np.sin(v))
        self.__z = wgs84.B * np.outer(np.ones_like(u), np.cos(v))

    def plot(self, ax: plt.Axes):
        if self.__obj is None:
            self.__obj = ax.plot_wireframe(self.__x, self.__y, self.__z, **self.__plot_kwargs)


class Wgs84PathPlot():
    def __init__(self, **plot_kwargs):
        self.__plot_kwargs = plot_kwargs
        self.__data_xyz: "list[np.ndarray]" = []
        self.__data_wgs84: "list[np.ndarray]" = []
        self.__lines3d = None
        self.__point3d = None
        self.__lines2d: Line2D = None
        self.__arrow2d: FancyArrow = None

    def append_xyz(self, xyz: "Iterable[float]"):
        self.__data_xyz.append(np.array(xyz))
        self.__data_wgs84.append(np.array(wgs84.xyz_to_wgs84(xyz)))

    def append_wgs84(self, wgs84: wgs84.Wgs84):
        self.__data_xyz.append(wgs84.to_xyz())
        self.__data_wgs84.append(np.array([wgs84.latitude, wgs84.longitude, wgs84.altitude]))

    def plot3d(self, ax: plt.Axes):
        if len(self.__data_xyz) < 2:
            return

        data = np.array(self.__data_xyz, dtype=np.float64)
        if self.__lines3d is None:
            self.__lines3d = ax.plot3D(data[:, 0], data[:, 1], data[:, 2], **self.__plot_kwargs)[0]
        else:
            self.__lines3d.set_data_3d(data[:, 0], data[:, 1], data[:, 2])

        # 最後の要素は矢印とする
        if self.__point3d is None:
            self.__point3d = ax.plot3D(data[-1, 0], data[-1, 1], data[-1, 2], marker='*', **self.__plot_kwargs)[0]
        else:
            self.__point3d.set_data_3d(data[-1, 0], data[-1, 1], data[-1, 2])

    def plot2d(self, ax: plt.Axes):
        if len(self.__data_wgs84) < 2:
            return

        # 線をプロットする
        line_data = np.array(self.__data_wgs84, dtype=np.float64)
        if self.__lines2d is None:
            self.__lines2d = ax.plot(line_data[:, 1], line_data[:, 0], **self.__plot_kwargs)[0]
        else:
            self.__lines2d.set_data(line_data[:, 1], line_data[:, 0])

        # 最後の要素の位置に矢印をプロットする
        arrow_dyx = line_data[-1, :] - line_data[-2, :]
        arrow_dyx *= 1.0 / np.linalg.norm(arrow_dyx)
        arrow_yx = line_data[-1, :] - arrow_dyx * 0.5
        if self.__arrow2d is None:
            self.__arrow2d = ax.arrow(arrow_yx[1], arrow_yx[0], arrow_dyx[1], arrow_dyx[0], width=0.5, length_includes_head=True, **self.__plot_kwargs)
        else:
            self.__arrow2d.set_data(x=arrow_yx[1], y=arrow_yx[0], dx=arrow_dyx[1], dy=arrow_dyx[0])


class PathPlot2d():
    def __init__(self, **plot_kwargs):
        self.__plot_kwargs = plot_kwargs
        self.__data_xyz: "list[np.ndarray]" = []
        self.__lines2d: Line2D = None
        self.__arrow2d: FancyArrow = None

    def append_xyz(self, xyz: "Iterable[float]"):
        self.__data_xyz.append(np.array(xyz))

    def plot2d(self, ax: plt.Axes):
        if len(self.__data_xyz) < 2:
            return

        # 線をプロットする
        line_data = np.array(self.__data_xyz, dtype=np.float64)
        if self.__lines2d is None:
            self.__lines2d = ax.plot(line_data[:, 0], line_data[:, 1], **self.__plot_kwargs)[0]
        else:
            self.__lines2d.set_data(line_data[:, 0], line_data[:, 1])

        # 最後の要素の位置に矢印をプロットする
        arrow_dxy = line_data[-1, :] - line_data[-2, :]
        arrow_dxy *= 1.0 / np.linalg.norm(arrow_dxy)
        arrow_xy = line_data[-1, :] - arrow_dxy * 0.5
        if self.__arrow2d is None:
            self.__arrow2d = ax.arrow(arrow_xy[0], arrow_xy[1], arrow_dxy[0], arrow_dxy[1], width=0.5, length_includes_head=True, **self.__plot_kwargs)
        else:
            self.__arrow2d.set_data(x=arrow_xy[0], y=arrow_xy[1], dx=arrow_dxy[0], dy=arrow_dxy[1])
