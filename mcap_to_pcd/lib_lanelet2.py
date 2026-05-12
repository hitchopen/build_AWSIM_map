"""Minimal Lanelet2 OSM writer for an Autoware-compatible single-lane HD map seed.

Produces an .osm with two ways (left/right lane boundaries), a centerline way,
and one lanelet relation linking them. Tags follow Autoware's convention so
the file loads cleanly into Autoware's lanelet2_map_loader and editors like
Vector Map Builder / JOSM (with lanelet2 plugin).

The trajectory is decimated by arc-length (one point every `step_m` metres)
and offset perpendicularly by `half_lane_width` metres to produce the lane
boundaries.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import List, Tuple

import numpy as np


def _decimate_by_arclength(positions: np.ndarray, step_m: float) -> np.ndarray:
    """Pick a subset of `positions` (Nx3) so consecutive picks are >= step_m apart along the path."""
    if positions.shape[0] == 0:
        return positions
    out = [positions[0]]
    last = positions[0]
    for p in positions[1:]:
        if np.linalg.norm(p - last) >= step_m:
            out.append(p)
            last = p
    if not np.array_equal(out[-1], positions[-1]):
        out.append(positions[-1])
    return np.array(out, dtype=np.float64)


def _perpendicular_offsets(centerline: np.ndarray, half_width: float
                           ) -> Tuple[np.ndarray, np.ndarray]:
    """Return (left, right) boundary lines, offset by half_width on each side.

    Direction is computed from finite differences. For each centerline point, we
    average the incoming and outgoing tangents to avoid kinks at sharp turns.
    The 2D normal is rotated 90° from the tangent: left is +90°, right is -90°.
    """
    n = centerline.shape[0]
    tangents = np.zeros_like(centerline)
    if n >= 2:
        tangents[1:-1] = centerline[2:] - centerline[:-2]
        tangents[0]    = centerline[1]   - centerline[0]
        tangents[-1]   = centerline[-1]  - centerline[-2]
    norms = np.linalg.norm(tangents[:, :2], axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    t2d = tangents[:, :2] / norms                               # (N, 2)
    # left normal = rotate t2d by +90°: (tx, ty) -> (-ty, tx)
    left_n = np.column_stack([-t2d[:, 1], t2d[:, 0]])
    left = centerline.copy()
    right = centerline.copy()
    left[:, :2]  += left_n * half_width
    right[:, :2] -= left_n * half_width
    return left, right


def _enu_to_geodetic(enu_xyz: np.ndarray, anchor) -> np.ndarray:
    """Inverse of ENUFrame.to_enu(). Uses small-angle approximation if anchor is plain dict;
    if it's an ENUFrame instance we use its rotation properly via ECEF round-trip."""
    if hasattr(anchor, '_R') and hasattr(anchor, '_origin_ecef'):
        # Proper inverse: ECEF = R^T @ enu + origin_ecef, then ECEF->geodetic
        R_inv = anchor._R.T
        origin = anchor._origin_ecef
        out = np.empty((enu_xyz.shape[0], 3), dtype=np.float64)
        for i, p in enumerate(enu_xyz):
            ecef = R_inv @ p + origin
            out[i] = _ecef_to_geodetic(*ecef)
        return out
    raise TypeError('anchor must be an ENUFrame instance')


def _ecef_to_geodetic(x: float, y: float, z: float) -> np.ndarray:
    """ECEF -> WGS84 geodetic (Bowring's method). Returns (lat_deg, lon_deg, alt_m)."""
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = f * (2 - f)
    b = a * (1 - f)
    ep2 = (a * a - b * b) / (b * b)
    p = math.hypot(x, y)
    th = math.atan2(z * a, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(z + ep2 * b * math.sin(th) ** 3,
                     p - e2 * a * math.cos(th) ** 3)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - N
    return np.array([math.degrees(lat), math.degrees(lon), alt], dtype=np.float64)


def write_lanelet2_osm(path: str,
                       trajectory_enu: np.ndarray,
                       enu_frame,
                       half_lane_width: float = 1.75,
                       step_m: float = 5.0) -> dict:
    """Write `path` as an Autoware-style Lanelet2 OSM.

    `trajectory_enu` is an (N, 3) array in the local ENU frame (metres). `enu_frame`
    is the ENUFrame used to anchor lat/lon for the OSM nodes (Lanelet2 OSM uses
    geodetic coordinates per node, and Autoware's projector reverses them).

    Returns a small summary dict.
    """
    centerline = _decimate_by_arclength(trajectory_enu, step_m)
    if centerline.shape[0] < 2:
        raise ValueError('trajectory too short to build a lanelet (need >= 2 decimated points)')
    left_enu, right_enu = _perpendicular_offsets(centerline, half_lane_width)

    # Convert each waypoint to lat/lon/alt
    left_geo   = _enu_to_geodetic(left_enu,   enu_frame)
    right_geo  = _enu_to_geodetic(right_enu,  enu_frame)
    centerline_geo = _enu_to_geodetic(centerline, enu_frame)

    osm = ET.Element('osm', attrib={'version': '0.6', 'generator': 'cowork-mcap-to-awsim'})
    osm.append(ET.Comment(' Map origin: lat={:.10f} lon={:.10f} alt={:.3f} '.format(
        enu_frame.lat0, enu_frame.lon0, enu_frame.alt0)))

    next_id = [1]

    def new_id() -> int:
        i = next_id[0]
        next_id[0] += 1
        return i

    def add_node(lat: float, lon: float, alt: float, ele_tag: bool = True) -> int:
        nid = new_id()
        n = ET.SubElement(osm, 'node', attrib={
            'id': str(nid), 'visible': 'true', 'version': '1',
            'lat': f'{lat:.12f}', 'lon': f'{lon:.12f}',
        })
        if ele_tag:
            ET.SubElement(n, 'tag', attrib={'k': 'ele', 'v': f'{alt:.3f}'})
        return nid

    def add_way(node_ids: List[int], tags: List[Tuple[str, str]]) -> int:
        wid = new_id()
        w = ET.SubElement(osm, 'way', attrib={'id': str(wid), 'visible': 'true', 'version': '1'})
        for nid in node_ids:
            ET.SubElement(w, 'nd', attrib={'ref': str(nid)})
        for k, v in tags:
            ET.SubElement(w, 'tag', attrib={'k': k, 'v': v})
        return wid

    def add_relation(members: List[Tuple[str, int, str]], tags: List[Tuple[str, str]]) -> int:
        rid = new_id()
        r = ET.SubElement(osm, 'relation', attrib={'id': str(rid), 'visible': 'true', 'version': '1'})
        for typ, ref, role in members:
            ET.SubElement(r, 'member', attrib={'type': typ, 'ref': str(ref), 'role': role})
        for k, v in tags:
            ET.SubElement(r, 'tag', attrib={'k': k, 'v': v})
        return rid

    left_node_ids  = [add_node(*g) for g in left_geo]
    right_node_ids = [add_node(*g) for g in right_geo]
    center_node_ids = [add_node(*g) for g in centerline_geo]

    left_way_id  = add_way(left_node_ids,  [
        ('type', 'line_thin'),
        ('subtype', 'solid'),
    ])
    right_way_id = add_way(right_node_ids, [
        ('type', 'line_thin'),
        ('subtype', 'solid'),
    ])
    center_way_id = add_way(center_node_ids, [
        ('type', 'centerline'),
    ])

    add_relation(
        members=[
            ('way', left_way_id,  'left'),
            ('way', right_way_id, 'right'),
            ('way', center_way_id, 'centerline'),
        ],
        tags=[
            ('type', 'lanelet'),
            ('subtype', 'road'),
            ('location', 'urban'),
            ('one_way', 'yes'),
            ('speed_limit', '30'),
            ('participant:vehicle', 'yes'),
        ],
    )

    tree = ET.ElementTree(osm)
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)

    return {
        'centerline_points': int(centerline.shape[0]),
        'half_lane_width_m': half_lane_width,
        'arclength_step_m':  step_m,
    }
