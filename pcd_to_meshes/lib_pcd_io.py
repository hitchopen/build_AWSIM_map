"""Reader for the binary PCD that mcap_to_pcd/build_map.py emits.

Format expected (matches lib_pcd.write_pcd_binary):
    FIELDS x y z intensity
    SIZE   4 4 4 4
    TYPE   F F F F
    COUNT  1 1 1 1
    DATA   binary

Plus a CSV trajectory loader for trajectory_enu.csv.

Pure numpy. No PCL, no Open3D.
"""
from __future__ import annotations

import csv
import os
from dataclasses import dataclass
from typing import Iterator, Optional, Tuple

import numpy as np


# ---------- PCD ---------------------------------------------------------------

def _parse_pcd_header(f) -> dict:
    """Read the ASCII header of a binary PCD. Leaves f positioned at the start
    of the binary payload (just after the trailing 'DATA binary\\n')."""
    hdr = {}
    while True:
        line = f.readline()
        if not line:
            raise ValueError('PCD: unexpected EOF in header')
        s = line.decode('ascii', errors='replace').strip()
        if s.startswith('#') or not s:
            continue
        key, _, val = s.partition(' ')
        hdr[key.upper()] = val.strip()
        if key.upper() == 'DATA':
            break
    return hdr


def read_pcd_xyzi(path: str) -> np.ndarray:
    """Read a binary PCD with fields x, y, z, intensity (all float32) and return
    an (N, 4) float32 array. Raises if the format is anything else.

    For a multi-hundred-MB PCD this just np.frombuffer() against the mmap'd
    payload — no per-point Python work."""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, 'rb') as f:
        hdr = _parse_pcd_header(f)
        if hdr.get('FIELDS', '').split() != ['x', 'y', 'z', 'intensity']:
            raise ValueError(f'PCD: unexpected FIELDS {hdr.get("FIELDS")!r}')
        if hdr.get('SIZE', '').split() != ['4', '4', '4', '4']:
            raise ValueError(f'PCD: unexpected SIZE {hdr.get("SIZE")!r}')
        if hdr.get('TYPE', '').split() != ['F', 'F', 'F', 'F']:
            raise ValueError(f'PCD: unexpected TYPE {hdr.get("TYPE")!r}')
        if hdr.get('DATA', '').lower() != 'binary':
            raise ValueError(f'PCD: only DATA binary supported, got {hdr.get("DATA")!r}')
        n = int(hdr.get('POINTS', hdr.get('WIDTH', 0)))
        if n <= 0:
            raise ValueError(f'PCD: POINTS not positive ({n})')
        raw = f.read(n * 16)
        if len(raw) != n * 16:
            raise ValueError(f'PCD: payload short, got {len(raw)} expected {n*16}')
    pts = np.frombuffer(raw, dtype=np.float32).reshape(n, 4).copy()
    return pts


# ---------- trajectory --------------------------------------------------------

def read_trajectory_csv(path: str) -> np.ndarray:
    """Read trajectory_enu.csv (t_ns, x_enu_m, y_enu_m, z_enu_m) and return an
    (N, 3) float64 array of XYZ in ENU."""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    xs, ys, zs = [], [], []
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        # Be permissive about column order
        try:
            ix = header.index('x_enu_m')
            iy = header.index('y_enu_m')
            iz = header.index('z_enu_m')
        except ValueError:
            ix, iy, iz = 1, 2, 3
        for row in r:
            xs.append(float(row[ix]))
            ys.append(float(row[iy]))
            zs.append(float(row[iz]))
    if not xs:
        raise ValueError(f'trajectory CSV empty: {path}')
    return np.column_stack([xs, ys, zs]).astype(np.float64)


def decimate_trajectory_arclength(traj: np.ndarray, step_m: float) -> np.ndarray:
    """Decimate so consecutive picks are >= step_m apart along the path.
    Drops near-duplicate samples that come from a 100 Hz INS sitting still."""
    if traj.shape[0] == 0:
        return traj
    keep = [traj[0]]
    last = traj[0]
    for p in traj[1:]:
        if np.linalg.norm(p - last) >= step_m:
            keep.append(p)
            last = p
    return np.array(keep, dtype=np.float64)


# ---------- corridor filter ---------------------------------------------------

@dataclass
class CorridorResult:
    """Output of `filter_to_corridor()`. Keep both the kept points and the
    nearest-trajectory-point info — the wall classifier needs the latter."""
    points: np.ndarray         # (M, 4) float32, kept (x, y, z, intensity)
    nearest_idx: np.ndarray    # (M,)  int32, index into trajectory
    signed_lat: np.ndarray     # (M,)  float32, signed lateral offset (left=+, right=-)
    along: np.ndarray          # (M,)  float32, arclength along trajectory


def filter_to_corridor(points_xyzi: np.ndarray,
                       trajectory: np.ndarray,
                       half_width_m: float,
                       max_height_above_road: float = 60.0,
                       max_depth_below_road: float = 10.0,
                       ) -> CorridorResult:
    """Keep only points whose horizontal distance to the trajectory polyline is
    <= half_width_m, and whose Z lies within [road_z - max_depth, road_z + max_height].

    Trajectory is treated as a polyline in 3D but distance is measured in 2D (XY)
    because the road's vertical extent is part of what we're trying to capture.

    Returns kept points + per-point nearest-trajectory-point metadata.
    The lateral sign is determined from the cross product of the trajectory
    tangent with the (point - traj) vector in 2D: left of travel = +, right = -.
    """
    from scipy.spatial import cKDTree

    if points_xyzi.shape[0] == 0:
        return CorridorResult(
            points=points_xyzi,
            nearest_idx=np.empty(0, dtype=np.int32),
            signed_lat=np.empty(0, dtype=np.float32),
            along=np.empty(0, dtype=np.float32),
        )
    if trajectory.shape[0] < 2:
        raise ValueError('trajectory needs at least 2 points')

    # Per-trajectory-point arclength — useful downstream
    seg = np.linalg.norm(np.diff(trajectory[:, :2], axis=0), axis=1)
    arclen = np.concatenate([[0.0], np.cumsum(seg)])  # (N_traj,)

    # Tangents (averaged neighbour difference, normalised in 2D)
    tan = np.zeros_like(trajectory)
    tan[1:-1] = trajectory[2:] - trajectory[:-2]
    tan[0]    = trajectory[1]  - trajectory[0]
    tan[-1]   = trajectory[-1] - trajectory[-2]
    tan2 = tan[:, :2]
    nrm = np.linalg.norm(tan2, axis=1, keepdims=True)
    nrm[nrm == 0] = 1.0
    tan2 = tan2 / nrm   # (N_traj, 2) unit

    # 2D KDTree on the (decimated) trajectory's XY
    kdt = cKDTree(trajectory[:, :2])

    # Process in chunks to bound memory
    chunk = 2_000_000
    keep_mask_parts = []
    near_idx_parts = []
    signed_lat_parts = []
    for start in range(0, points_xyzi.shape[0], chunk):
        sl = slice(start, start + chunk)
        xy = points_xyzi[sl, :2].astype(np.float64)
        z  = points_xyzi[sl, 2].astype(np.float64)
        d, i = kdt.query(xy, k=1, distance_upper_bound=half_width_m + 0.5)
        within_lat = d <= half_width_m
        # For the height gate, compare to the trajectory's Z at the nearest point
        i_safe = np.where(within_lat, i, 0)  # avoid OOB read on i==N (kdt's "no-neighbour")
        traj_z = trajectory[i_safe, 2]
        within_z = (z <= traj_z + max_height_above_road) & (z >= traj_z - max_depth_below_road)
        keep = within_lat & within_z

        # Signed lateral offset: cross(tangent, (p - traj)) in 2D
        diff_xy = xy - trajectory[i_safe, :2]
        t_here = tan2[i_safe]
        cross = t_here[:, 0] * diff_xy[:, 1] - t_here[:, 1] * diff_xy[:, 0]
        # In a right-handed XY frame with X east, Y north, "left of travel" gives
        # cross > 0; "right" gives cross < 0. Magnitude = perpendicular distance.

        keep_mask_parts.append(keep)
        near_idx_parts.append(i_safe.astype(np.int32))
        signed_lat_parts.append(cross.astype(np.float32))

    keep_mask = np.concatenate(keep_mask_parts)
    near_idx  = np.concatenate(near_idx_parts)
    signed_lat = np.concatenate(signed_lat_parts)

    pts_kept = points_xyzi[keep_mask]
    near_idx_kept = near_idx[keep_mask]
    signed_lat_kept = signed_lat[keep_mask]
    along_kept = arclen[near_idx_kept].astype(np.float32)

    return CorridorResult(
        points=pts_kept,
        nearest_idx=near_idx_kept,
        signed_lat=signed_lat_kept,
        along=along_kept,
    )
