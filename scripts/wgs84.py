#!/usr/bin/python3

import numpy
import math
from typing import Iterable


A = 6378137.0
"""長軸の半径"""

F = 1 / 298.257223563
"""扁平率"""

E2 = 2 * F - F * F
"""離心率"""

B = A * (1 - F)
"""短軸の半径"""


def wgs84_to_xyz(lat: float = 0.0, lon: float = 0.0, alt: float = 0.0) -> numpy.ndarray:
    phi = lat * math.pi / 180
    theta = lon * math.pi / 180
    h = alt
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)
    sin_phi_2 = sin_phi * sin_phi
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    n = A / math.sqrt(1.0 - E2 * sin_phi_2)
    x = (n + h) * cos_phi * cos_theta
    y = (n + h) * cos_phi * sin_theta
    z = (n * (1.0 - E2) + h) * sin_phi
    return numpy.array([x, y, z])


def xyz_to_wgs84(xyz: "Iterable[float]") -> "tuple[float]":
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]
    p = numpy.linalg.norm(xyz[0:2])
    r = numpy.linalg.norm(xyz[0:3])
    mu = math.atan((z / p) * ((1.0 - F) + E2 * A / r))
    cos_mu = math.cos(mu)
    sin_mu = math.sin(mu)
    cos_mu_3 = cos_mu * cos_mu * cos_mu
    sin_mu_3 = sin_mu * sin_mu * sin_mu
    phi = math.atan((z * (1.0 - F) + E2 * A * sin_mu_3) / ((1.0 - F) * (p - E2 * A * cos_mu_3)))
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)
    sin_phi_2 = sin_phi * sin_phi
    h = p * cos_phi + z * sin_phi - A * math.sqrt(1.0 - E2 * sin_phi_2)
    theta = math.atan2(y, x)
    return phi * 180 / math.pi, theta * 180 / math.pi, h


class Wgs84:
    def __init__(self, lat: float = 0.0, lon: float = 0.0, alt: float = 0.0):
        self.latitude = lat
        self.longitude = lon
        self.altitude = alt

    def to_xyz(self) -> numpy.ndarray:
        return wgs84_to_xyz(self.latitude, self.longitude, self.altitude)

    def normal_vector(self) -> numpy.ndarray:
        return numpy.array([
            math.cos(self.latitude * math.pi / 180) * math.cos(self.longitude * math.pi / 180),
            math.cos(self.latitude * math.pi / 180) * math.sin(self.longitude * math.pi / 180),
            math.sin(self.latitude * math.pi / 180)
        ])

    @staticmethod
    def from_xyz(xyz: numpy.ndarray):
        return Wgs84(*xyz_to_wgs84(xyz))
