"""Trajectory extraction and frame conversions.

Coordinate frames used here:
  - WGS84:  geodetic lat/lon/alt
  - ENU:    local Cartesian — X east, Y north, Z up — origin at a chosen lat/lon/alt
  - NED:    Novatel SPAN local-level — X north, Y east, Z down
  - body (FRD): SPAN body — X forward, Y right, Z down
  - lidar (FLU): Hesai sensor — X forward, Y left, Z up

The /pp7/inspvax message gives us (lat, lon, alt, roll, pitch, azimuth) in WGS84
+ NED-style angles. We convert each pose to (x, y, z) in local ENU and a 3x3
rotation matrix that takes a point from `lidar` frame straight into ENU world.
"""
from __future__ import annotations
import bisect
import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np


# WGS84 constants for geodetic <-> ENU
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2 - _WGS84_F)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """WGS84 geodetic -> ECEF."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sl * sl)
    x = (N + alt_m) * cl * math.cos(lon)
    y = (N + alt_m) * cl * math.sin(lon)
    z = (N * (1 - _WGS84_E2) + alt_m) * sl
    return np.array([x, y, z], dtype=np.float64)


class ENUFrame:
    """Local tangent ENU frame anchored at a geodetic origin."""

    def __init__(self, lat0_deg: float, lon0_deg: float, alt0_m: float):
        self.lat0 = lat0_deg
        self.lon0 = lon0_deg
        self.alt0 = alt0_m
        self._origin_ecef = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_m)
        lat = math.radians(lat0_deg)
        lon = math.radians(lon0_deg)
        sl, cl = math.sin(lat), math.cos(lat)
        so, co = math.sin(lon), math.cos(lon)
        # Rotation ECEF -> ENU
        self._R = np.array([
            [-so,        co,       0.0],
            [-sl * co,  -sl * so,  cl],
            [cl * co,   cl * so,   sl],
        ], dtype=np.float64)

    def to_enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
        ecef = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
        return self._R @ (ecef - self._origin_ecef)


# Static rotations between frames
_R_NED_TO_ENU = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)

# LiDAR (FLU) -> body (FRD): flip Y and Z
_R_LIDAR_TO_BODY = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
], dtype=np.float64)


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def body_to_ned(roll_deg: float, pitch_deg: float, azimuth_deg: float) -> np.ndarray:
    """Rotation that takes a point in the SPAN body frame (FRD) into NED local-level.

    SPAN convention (per OEM7 INS manual): the rotation from body to local-level
    is R_z(azimuth) * R_y(pitch) * R_x(roll), with axes Z=down, Y=east, X=north.
    """
    return _rot_z(math.radians(azimuth_deg)) @ \
           _rot_y(math.radians(pitch_deg)) @ \
           _rot_x(math.radians(roll_deg))


def lidar_to_enu_rotation(roll_deg: float, pitch_deg: float, azimuth_deg: float,
                          extra_R_lidar_to_body: Optional[np.ndarray] = None) -> np.ndarray:
    """Compose: LiDAR (FLU) -> body (FRD) -> NED -> ENU.

    `extra_R_lidar_to_body` lets the caller override the default Y/Z flip,
    e.g. for a non-default LiDAR mounting.
    """
    R_lid_to_body = extra_R_lidar_to_body if extra_R_lidar_to_body is not None else _R_LIDAR_TO_BODY
    R_body_to_ned = body_to_ned(roll_deg, pitch_deg, azimuth_deg)
    return _R_NED_TO_ENU @ R_body_to_ned @ R_lid_to_body


@dataclass
class Pose:
    t_ns: int
    pos: np.ndarray              # (3,) ENU position of IMU
    R_lidar_to_world: np.ndarray # (3,3) rotation lidar -> ENU


class TrajectoryStore:
    """Sorted list of poses with timestamp-based nearest-neighbor lookup.

    Lookup is O(log n) via bisect on the timestamp array. The store assumes
    monotonically-increasing timestamps (true for a typical bag).
    """

    def __init__(self):
        self._t: List[int] = []
        self._poses: List[Pose] = []

    def append(self, p: Pose) -> None:
        self._t.append(p.t_ns)
        self._poses.append(p)

    def __len__(self) -> int:
        return len(self._poses)

    def empty(self) -> bool:
        return not self._poses

    def all_positions(self) -> np.ndarray:
        return np.array([p.pos for p in self._poses], dtype=np.float64)

    def lookup(self, t_ns: int, max_dt_ns: int = int(0.05e9)) -> Optional[Pose]:
        """Return pose closest to t_ns (within max_dt_ns), or None."""
        if not self._poses:
            return None
        i = bisect.bisect_left(self._t, t_ns)
        candidates = []
        if i < len(self._poses):
            candidates.append(self._poses[i])
        if i > 0:
            candidates.append(self._poses[i - 1])
        best = min(candidates, key=lambda p: abs(p.t_ns - t_ns))
        if abs(best.t_ns - t_ns) > max_dt_ns:
            return None
        return best
