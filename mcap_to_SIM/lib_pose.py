"""Trajectory extraction, frame conversions, and /tf_static loader.

Frames in this build:
  - WGS84:    geodetic lat / lon / alt
  - ENU:      local Cartesian — X east, Y north, Z up — world frame for the map
  - "imu":    bag's IMU frame, FLU (X forward, Y left, Z up). This is the
              reference frame in sensor_tf_no_camera.yaml and the parent of
              every /tf_static entry. INSPVAX, RawImu, EKF pose, NavSatFix
              all carry header.frame_id = 'imu'.
  - lidar:    each lidar's own sensor frame. The static transform from this
              frame to "imu" is read from the bag's /tf_static (or supplied
              via the YAML), per the cleaned_tf.mcap that inject_tf_static.py
              produces.

Subtle bit: even though the bag's "imu" frame is FLU, the INSPVAX message's
(roll, pitch, azimuth) fields are in Novatel SPAN's native convention, which
describes a FRD body in NED local-level. So the conversion still has to
account for that — see `body_FRD_to_ned` and `imu_to_enu_rotation`.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------- WGS84 ↔ ECEF / ENU --------------------------------------------------

_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2 - _WGS84_F)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """WGS84 geodetic → ECEF."""
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
        self._R = np.array([
            [-so,        co,       0.0],
            [-sl * co,  -sl * so,  cl],
            [cl * co,   cl * so,   sl],
        ], dtype=np.float64)

    def to_enu(self, lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
        ecef = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
        return self._R @ (ecef - self._origin_ecef)


# ---------- static rotation primitives -----------------------------------------

# NED ↔ ENU is just an axis swap + Z flip
_R_NED_TO_ENU = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)

# FLU body (X fwd, Y left, Z up) ↔ FRD body (X fwd, Y right, Z down).
# Same matrix in both directions (involutory).
R_FLU_FRD_FLIP = np.diag([1.0, -1.0, -1.0])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def body_FRD_to_ned(roll_deg: float, pitch_deg: float, azimuth_deg: float
                    ) -> np.ndarray:
    """Rotation from SPAN's FRD body frame to NED local-level, given the SPAN
    INSPVAX angle triplet (roll, pitch, azimuth in degrees).

    Convention (per OEM7 INS manual): R_z(azimuth) · R_y(pitch) · R_x(roll).
    """
    return (_rot_z(math.radians(azimuth_deg))
            @ _rot_y(math.radians(pitch_deg))
            @ _rot_x(math.radians(roll_deg)))


def imu_to_enu_rotation(roll_deg: float, pitch_deg: float, azimuth_deg: float
                        ) -> np.ndarray:
    """Rotation that takes a vector in the bag's "imu" frame (FLU body) into
    ENU world. Builds the SPAN→FLU axis flip into the chain so SPAN's FRD-style
    angles still produce the correct world rotation.

      R_imu_FLU → ENU
        = R_NED→ENU  ·  R_FRD→NED(roll, pitch, azimuth)  ·  R_FLU→FRD
    """
    return _R_NED_TO_ENU @ body_FRD_to_ned(roll_deg, pitch_deg, azimuth_deg) @ R_FLU_FRD_FLIP


def lidar_to_enu_rotation(roll_deg: float, pitch_deg: float, azimuth_deg: float,
                          R_lidar_to_imu: np.ndarray) -> np.ndarray:
    """Compose the dynamic IMU→ENU rotation with a static lidar→IMU rotation
    (typically read from /tf_static)."""
    return imu_to_enu_rotation(roll_deg, pitch_deg, azimuth_deg) @ R_lidar_to_imu


# ---------- /tf_static loader --------------------------------------------------

@dataclass
class StaticTransform:
    """A single TransformStamped entry from /tf_static.

    `R` and `t` give the child→parent transform in the convention
        p_parent = R @ p_child + t
    """
    parent: str
    child: str
    R: np.ndarray   # (3, 3)
    t: np.ndarray   # (3,)


def _quat_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),       2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def read_tf_static(mcap_path: str) -> Dict[str, StaticTransform]:
    """Read the first /tf_static message from the bag and return a dict
    keyed by `child_frame_id`. Returns an empty dict if the topic isn't
    present (caller should error or warn)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    out: Dict[str, StaticTransform] = {}
    with open(mcap_path, 'rb') as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for sch, ch, msg, decoded in reader.iter_decoded_messages(topics=['/tf_static']):
            for tx in decoded.transforms:
                t = np.array([tx.transform.translation.x,
                              tx.transform.translation.y,
                              tx.transform.translation.z], dtype=np.float64)
                q = tx.transform.rotation
                R = _quat_to_R(q.x, q.y, q.z, q.w)
                out[tx.child_frame_id] = StaticTransform(
                    parent=tx.header.frame_id, child=tx.child_frame_id, R=R, t=t)
            break  # /tf_static is latched; one message is enough
    return out


def peek_topic_frame_id(mcap_path: str, topic: str) -> Optional[str]:
    """Pull a single message off `topic` and return its header.frame_id."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    with open(mcap_path, 'rb') as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for sch, ch, msg, decoded in reader.iter_decoded_messages(topics=[topic]):
            return decoded.header.frame_id
    return None


# ---------- pose store ---------------------------------------------------------

@dataclass
class Pose:
    t_ns: int
    pos: np.ndarray             # (3,) ENU position of "imu" frame origin
    R_imu_to_world: np.ndarray  # (3, 3) rotation taking imu/FLU vectors → ENU


class TrajectoryStore:
    """Sorted list of poses with timestamp-based nearest-neighbour lookup
    (O(log n) via bisect). Assumes monotonically-increasing timestamps."""

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
