#!/usr/bin/env python3
"""Build an AWSIM track map (PCD + Lanelet2 + map_origin) from cleaned_tf.mcap.

Two passes:

  1. Pose pass — read every /pp7/inspvax message, convert lat/lon/alt to
     local ENU and (roll, pitch, azimuth) to a rotation that takes a vector
     in the bag's "imu" frame (FLU) into ENU world. PROPOGATED INS fixes
     are dropped by default.

  2. LiDAR pass — for each (decimated) scan from each lidar topic:
       * range-filter raw points in the lidar's own frame,
       * transform to "imu" using the static lidar→imu transform read from
         /tf_static,
       * drop points inside the ego bounding box (in IMU frame),
       * apply the dynamic R_imu→ENU and the IMU's ENU position,
       * voxel-downsample on the fly.

Outputs (under --out-dir):
    pointcloud_map.pcd      binary PCD (x, y, z, intensity float32)
    lanelet2_map.osm        seed Lanelet2 (centerline + ±half-width boundaries)
    map_origin.yaml         lat / lon / alt of ENU origin
    trajectory_enu.csv      decimated ego trajectory (debug aid)

Run:
    python3 build_map.py \\
        --mcap ../data/TM99_uphill/cleaned_tf.mcap \\
        --out-dir ../map/TM99_uphill
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib_pose import (
    ENUFrame, Pose, TrajectoryStore,
    StaticTransform, imu_to_enu_rotation, peek_topic_frame_id, read_tf_static,
)
from lib_pcd import StreamingVoxelMap, write_pcd_binary
from lib_lanelet2 import write_lanelet2_osm

from mcap.reader import make_reader
from mcap_ros2._dynamic import generate_dynamic


INSPVAX_TYPE     = 'starneto_gps_msgs/msg/Inspvax'
POINTCLOUD2_TYPE = 'sensor_msgs/msg/PointCloud2'
INSPVAX_TOPIC    = '/pp7/inspvax'

DEFAULT_LIDAR_TOPICS = ('/lidar_points_compressed', '/lidar_points_2_compressed')

# Default ego bounding box in IMU frame (FLU, metres). A typical hillclimb
# car is ~5 m long × 2 m wide × 1.5 m tall; the IMU sits roughly mid-cabin,
# slightly above ground. These defaults should clip the chassis, hood and
# roof returns from both lidars without nipping into the road surface or
# adjacent walls. Override per-axis with --ego-bbox.
DEFAULT_EGO_BBOX = (-3.0, +3.0,   # x (forward / rearward)
                    -1.2, +1.2,   # y (left   / right)
                    -1.8, +0.5)   # z (up=+ ; below IMU origin = -)


# ---------- args --------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--mcap', required=True, help='input MCAP bag (cleaned_tf.mcap)')
    p.add_argument('--out-dir', required=True, help='where to write PCD + OSM + yaml')

    # Origin
    p.add_argument('--origin-lat', type=float, default=None,
                   help='ENU origin latitude (deg). Default: first usable INS fix.')
    p.add_argument('--origin-lon', type=float, default=None,
                   help='ENU origin longitude (deg). Default: first usable INS fix.')
    p.add_argument('--origin-alt', type=float, default=None,
                   help='ENU origin altitude (m, ellipsoidal). Default: first usable INS fix.')

    # PCD
    p.add_argument('--voxel-size', type=float, default=0.2,
                   help='PCD voxel grid (m). Default 0.2.')
    p.add_argument('--lidar-topics', default=','.join(DEFAULT_LIDAR_TOPICS),
                   help='Comma-separated list of PointCloud2 topics to fuse. '
                        'Default: both Hesai lidars from cleaned_tf.mcap.')
    p.add_argument('--lidar-decimate', type=int, default=5,
                   help='Use every Nth scan per lidar. Default 5 (≈2 Hz from 10 Hz source).')

    # Range filter (lidar frame)
    p.add_argument('--min-range', type=float, default=2.0,
                   help='Drop points closer than this (m) from the sensor (lidar frame).')
    p.add_argument('--max-range', type=float, default=120.0,
                   help='Drop points farther than this (m) from the sensor.')

    # Ego bounding box (IMU/FLU frame)
    p.add_argument('--ego-bbox', default=','.join(f'{v:+.2f}' for v in DEFAULT_EGO_BBOX),
                   help='Bounding box in IMU/FLU coordinates that excludes returns from '
                        'the ego vehicle itself: "x_min,x_max,y_min,y_max,z_min,z_max" (metres). '
                        'Default: ' + ','.join(f'{v:+.2f}' for v in DEFAULT_EGO_BBOX) + '. '
                        'Pass "none" to disable.')

    # Z gate in ENU (catches sensor glitches, sky points, etc.)
    # NOTE: defaults are RELATIVE to the trajectory's altitude range, not to
    # the ENU origin. A fixed cap (e.g. +300) silently truncates lidar coverage
    # on tracks with elevation gain greater than the cap — see README.
    p.add_argument('--min-z-margin', type=float, default=10.0,
                   help='Keep ENU points down to (trajectory_z_min − this many m). Default 10.')
    p.add_argument('--max-z-margin', type=float, default=100.0,
                   help='Keep ENU points up to (trajectory_z_max + this many m). Default 100 — '
                        'enough to capture cliff face above the road on steep climbs.')
    p.add_argument('--min-z', type=float, default=None,
                   help='Override the auto-computed lower Z cutoff (absolute ENU m). '
                        'If set, --min-z-margin is ignored.')
    p.add_argument('--max-z', type=float, default=None,
                   help='Override the auto-computed upper Z cutoff (absolute ENU m). '
                        'If set, --max-z-margin is ignored.')

    # Trajectory filtering
    p.add_argument('--accept-propagated', action='store_true',
                   help='Accept INS_PROPOGATED fixes (lower quality). Default: RTK-class only.')
    p.add_argument('--max-pose-dt-ms', type=int, default=50,
                   help='Reject a scan if no pose lies within this many ms.')

    # Lanelet2
    p.add_argument('--half-lane-width', type=float, default=1.75)
    p.add_argument('--lanelet-step-m', type=float, default=5.0)
    p.add_argument('--skip-lanelet2', action='store_true')

    # Debug
    p.add_argument('--max-scans', type=int, default=None,
                   help='Hard cap on lidar scans (per topic) processed. Default: unlimited.')
    p.add_argument('--save-ego-debug-pcd', action='store_true',
                   help='Write a separate ego_returns.pcd containing the points that the '
                        'bbox filter rejected (in ENU world). Useful to confirm the bbox '
                        'really cuts vehicle structure rather than environment.')
    return p.parse_args()


# ---------- pose pass ----------------------------------------------------------

ACCEPTABLE_POSITION_TYPES_RTK = {
    'INS_RTKFIXED', 'INS_RTKFLOAT', 'INS_PSRDIFF', 'INS_PSRSP',
    'NARROW_INT', 'NARROW_FLOAT', 'L1_FLOAT', 'WIDE_INT', 'IONOFREE_FLOAT',
}


def extract_trajectory(mcap_path: str, accept_propagated: bool,
                       origin_override) -> tuple[ENUFrame, TrajectoryStore]:
    print(f'[pose] reading {mcap_path}', flush=True)
    t0 = time.time()
    raw_traj: list[tuple[int, float, float, float, float, float, float]] = []
    with open(mcap_path, 'rb') as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        sch = next(s for s in summary.schemas.values() if s.name == INSPVAX_TYPE)
        decoder = generate_dynamic(sch.name, sch.data.decode())[sch.name]

        n_total = sum(c for cid, c in summary.statistics.channel_message_counts.items()
                      if summary.channels[cid].topic == INSPVAX_TOPIC)
        last_print = t0
        n_seen = 0
        for sch2, ch, msg in reader.iter_messages(topics=[INSPVAX_TOPIC]):
            n_seen += 1
            d = decoder(msg.data)
            if not accept_propagated and d.position_type not in ACCEPTABLE_POSITION_TYPES_RTK:
                continue
            raw_traj.append((msg.log_time, d.latitude, d.longitude, d.altitude,
                             d.roll, d.pitch, d.azimuth))
            now = time.time()
            if now - last_print > 5.0:
                print(f'  [pose] {n_seen}/{n_total} read, {len(raw_traj)} kept   '
                      f'rate {n_seen/(now-t0):.0f} msg/s', flush=True)
                last_print = now

    if not raw_traj:
        raise SystemExit('no usable INSPVAX poses found (try --accept-propagated)')

    print(f'[pose] kept {len(raw_traj)} of {n_seen} INSPVAX poses '
          f'({100.0*len(raw_traj)/max(n_seen,1):.1f}%) in {time.time()-t0:.1f}s', flush=True)

    if origin_override[0] is not None:
        lat0, lon0, alt0 = origin_override
        print(f'[pose] using user-specified origin: lat={lat0:.7f} lon={lon0:.7f} alt={alt0:.2f}',
              flush=True)
    else:
        _, lat0, lon0, alt0, *_ = raw_traj[0]
        print(f'[pose] using first usable INS fix as origin: '
              f'lat={lat0:.7f} lon={lon0:.7f} alt={alt0:.2f}', flush=True)
    enu = ENUFrame(lat0, lon0, alt0)

    store = TrajectoryStore()
    for t_ns, lat, lon, alt, roll, pitch, az in raw_traj:
        store.append(Pose(
            t_ns=t_ns,
            pos=enu.to_enu(lat, lon, alt),
            R_imu_to_world=imu_to_enu_rotation(roll, pitch, az),
        ))
    print(f'[pose] built trajectory store: {len(store)} poses', flush=True)
    return enu, store


# ---------- lidar pass --------------------------------------------------------

def parse_pointcloud2(d) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Decode a sensor_msgs/PointCloud2 with fields x,y,z (f32), intensity (f32),
    ring (u16), timestamp (f64). Returns (x, y, z, intensity) as float32 arrays."""
    n = d.width * d.height
    step = d.point_step
    if n == 0 or step == 0:
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty, empty
    arr = np.frombuffer(bytes(d.data), dtype=np.uint8)[: n * step].reshape(n, step)
    xs = np.frombuffer(arr[:, 0:4].tobytes(), dtype=np.float32)
    ys = np.frombuffer(arr[:, 4:8].tobytes(), dtype=np.float32)
    zs = np.frombuffer(arr[:, 8:12].tobytes(), dtype=np.float32)
    ii = np.frombuffer(arr[:, 12:16].tobytes(), dtype=np.float32)
    return xs, ys, zs, ii


def parse_ego_bbox(s: str) -> tuple[float, float, float, float, float, float] | None:
    if s.lower() in ('none', 'off', ''):
        return None
    parts = [float(x.strip()) for x in s.split(',')]
    if len(parts) != 6:
        raise SystemExit(f'--ego-bbox needs 6 comma-separated values, got {len(parts)}')
    x_min, x_max, y_min, y_max, z_min, z_max = parts
    if x_min >= x_max or y_min >= y_max or z_min >= z_max:
        raise SystemExit('--ego-bbox: each (min, max) pair must satisfy min < max')
    return (x_min, x_max, y_min, y_max, z_min, z_max)


def _ego_inside_mask(pts_imu: np.ndarray,
                      bbox: tuple[float, float, float, float, float, float]
                      ) -> np.ndarray:
    """Boolean mask, True for points INSIDE the ego bbox in IMU frame."""
    x_min, x_max, y_min, y_max, z_min, z_max = bbox
    return ((pts_imu[:, 0] >= x_min) & (pts_imu[:, 0] <= x_max) &
            (pts_imu[:, 1] >= y_min) & (pts_imu[:, 1] <= y_max) &
            (pts_imu[:, 2] >= z_min) & (pts_imu[:, 2] <= z_max))


# Per-topic counters for the diagnostic report
class TopicCounters:
    __slots__ = ('scans', 'raw_points', 'after_range', 'after_ego', 'after_z')
    def __init__(self):
        self.scans = 0
        self.raw_points = 0
        self.after_range = 0
        self.after_ego = 0
        self.after_z = 0


def build_pcd(mcap_path: str,
              enu: ENUFrame,
              traj: TrajectoryStore,
              tf_static: dict,
              args) -> tuple[StreamingVoxelMap, dict, StreamingVoxelMap | None]:
    """Stream every requested lidar topic into one voxel-downsampled accumulator.

    Returns (vmap, per_topic_counters, ego_debug_vmap).
    `ego_debug_vmap` is None unless --save-ego-debug-pcd was set; otherwise it's
    a separate accumulator collecting only the points that the ego bbox rejected
    (in ENU world coords) — handy for visually confirming the bbox actually cut
    vehicle returns and not real environment."""
    topics = [t.strip() for t in args.lidar_topics.split(',') if t.strip()]
    if not topics:
        raise SystemExit('--lidar-topics is empty')

    bbox = parse_ego_bbox(args.ego_bbox)
    if bbox is not None:
        print(f'[pcd] ego bbox in IMU/FLU: '
              f'x∈[{bbox[0]:+.2f}, {bbox[1]:+.2f}], '
              f'y∈[{bbox[2]:+.2f}, {bbox[3]:+.2f}], '
              f'z∈[{bbox[4]:+.2f}, {bbox[5]:+.2f}] m', flush=True)
    else:
        print(f'[pcd] ego bbox filter: DISABLED (--ego-bbox=none)', flush=True)

    # For each topic, peek its frame_id and look up the static transform.
    per_topic_R = {}  # topic -> (R_lidar_to_imu, t_lidar_to_imu, frame_id)
    print(f'[pcd] lidar topics:', flush=True)
    for topic in topics:
        frame_id = peek_topic_frame_id(mcap_path, topic)
        if frame_id is None:
            print(f'    {topic:42s} → no messages, skipping', flush=True)
            continue
        if frame_id not in tf_static:
            print(f'    {topic:42s} frame_id={frame_id!r} → no /tf_static entry; '
                  f'falling back to identity', flush=True)
            per_topic_R[topic] = (np.eye(3), np.zeros(3), frame_id)
            continue
        tf = tf_static[frame_id]
        per_topic_R[topic] = (tf.R, tf.t, frame_id)
        print(f'    {topic:42s} frame_id={frame_id!r}  '
              f't=({tf.t[0]:+.3f}, {tf.t[1]:+.3f}, {tf.t[2]:+.3f}) m  '
              f'parent={tf.parent}', flush=True)

    if not per_topic_R:
        raise SystemExit('no lidar topics could be set up')

    vmap = StreamingVoxelMap(voxel_size=args.voxel_size)
    ego_dbg = (StreamingVoxelMap(voxel_size=args.voxel_size)
               if args.save_ego_debug_pcd else None)

    max_dt_ns = int(args.max_pose_dt_ms * 1e6)
    min_r2 = args.min_range ** 2
    max_r2 = args.max_range ** 2

    counters = {topic: TopicCounters() for topic in per_topic_R}

    with open(mcap_path, 'rb') as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        sch = next(s for s in summary.schemas.values() if s.name == POINTCLOUD2_TYPE)
        pc2_dec = generate_dynamic(sch.name, sch.data.decode())[sch.name]

        idx_per_topic = {topic: -1 for topic in per_topic_R}
        n_processed = 0
        n_skipped_no_pose = 0
        t0 = time.time()
        last_print = t0

        for sch2, ch, msg in reader.iter_messages(topics=list(per_topic_R)):
            topic = ch.topic
            idx_per_topic[topic] += 1
            if idx_per_topic[topic] % args.lidar_decimate != 0:
                continue
            if args.max_scans is not None and counters[topic].scans >= args.max_scans:
                continue

            pose = traj.lookup(msg.log_time, max_dt_ns=max_dt_ns)
            if pose is None:
                n_skipped_no_pose += 1
                continue

            R_static, t_static, _ = per_topic_R[topic]
            d = pc2_dec(msg.data)
            xs, ys, zs, ii = parse_pointcloud2(d)
            if xs.size == 0:
                continue
            counters[topic].scans += 1
            counters[topic].raw_points += xs.size

            # Range filter in lidar frame
            r2 = xs * xs + ys * ys + zs * zs
            mask = (r2 >= min_r2) & (r2 <= max_r2) & np.isfinite(r2)
            if not mask.any():
                continue
            xs, ys, zs, ii = xs[mask], ys[mask], zs[mask], ii[mask]
            counters[topic].after_range += xs.size

            local = np.column_stack([xs, ys, zs]).astype(np.float64)
            pts_imu = (R_static @ local.T).T + t_static                # (N, 3) IMU frame

            # Ego bbox filter in IMU frame
            if bbox is not None:
                inside = _ego_inside_mask(pts_imu, bbox)
                keep = ~inside
                if ego_dbg is not None and inside.any():
                    rej_imu = pts_imu[inside]
                    rej_world = (pose.R_imu_to_world @ rej_imu.T).T + pose.pos
                    ego_dbg.add(np.column_stack([rej_world.astype(np.float32),
                                                  ii[inside].astype(np.float32)]))
                pts_imu = pts_imu[keep]
                ii = ii[keep]
                if pts_imu.shape[0] == 0:
                    continue
            counters[topic].after_ego += pts_imu.shape[0]

            # Dynamic IMU→ENU + IMU position
            world = (pose.R_imu_to_world @ pts_imu.T).T + pose.pos     # (N, 3) ENU

            # Z gate (catches occasional sky/sensor-glitch points)
            zmask = (world[:, 2] >= args.min_z) & (world[:, 2] <= args.max_z)
            if not zmask.any():
                continue
            world = world[zmask]
            ii = ii[zmask]
            counters[topic].after_z += world.shape[0]

            vmap.add(np.column_stack([world.astype(np.float32),
                                       ii.astype(np.float32, copy=False)]))
            n_processed += 1

            now = time.time()
            if now - last_print > 5.0:
                rate = n_processed / max(now - t0, 0.001)
                stats = vmap.stats()
                print(f'  [pcd] scans {n_processed:6d} from {len(per_topic_R)} topic(s) '
                      f' rate {rate:.1f} sc/s  voxels {stats["voxels"]:>8,}  '
                      f'pending {stats["pending"]:>9,}', flush=True)
                last_print = now

    print(f'[pcd] processed {n_processed} scans in {time.time()-t0:.1f}s '
          f'({n_skipped_no_pose} skipped, no pose)', flush=True)
    return vmap, counters, ego_dbg


def print_filter_report(counters: dict) -> None:
    print(f'\n[filter] per-topic point counts (after each filter stage):')
    print(f'  {"topic":42s} {"scans":>8s}  {"raw":>14s}  {"after range":>14s}  '
          f'{"after ego":>14s}  {"after Z":>14s}  {"% kept":>7s}')
    for topic, c in counters.items():
        kept_pct = 100.0 * c.after_z / max(c.raw_points, 1)
        ego_filt = c.after_range - c.after_ego
        print(f'  {topic:42s} {c.scans:>8,}  {c.raw_points:>14,}  '
              f'{c.after_range:>14,}  {c.after_ego:>14,}  {c.after_z:>14,}  '
              f'{kept_pct:>6.1f}%')
        if c.after_range > 0:
            print(f'  {"  -> ego bbox removed":42s} '
                  f'{ego_filt:>14,} pts ({100.0*ego_filt/c.after_range:.1f}% of in-range)')


# ---------- main --------------------------------------------------------------

def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    origin = (args.origin_lat, args.origin_lon, args.origin_alt)
    if any(v is None for v in origin) and not all(v is None for v in origin):
        raise SystemExit('--origin-lat/-lon/-alt must all be supplied or all be omitted')

    # /tf_static
    print(f'[tf  ] reading /tf_static from bag...', flush=True)
    tf_static = read_tf_static(args.mcap)
    if not tf_static:
        raise SystemExit(
            f'No /tf_static found in {args.mcap}. Run inject_tf_static.py first '
            f'to bake sensor_tf_no_camera.yaml into the bag, then re-run.')
    for child, tf in tf_static.items():
        print(f'    {tf.parent} -> {child}  '
              f't=({tf.t[0]:+.3f}, {tf.t[1]:+.3f}, {tf.t[2]:+.3f}) m', flush=True)

    enu, traj = extract_trajectory(args.mcap, args.accept_propagated, origin)

    # Resolve the Z gate against the trajectory's actual altitude range. A
    # fixed default like --max-z=+300 will silently clip lidar coverage on any
    # track whose elevation gain exceeds the cap (e.g. TM99 climbs ~715 m, so
    # +300 used to drop ~60% of the climb's lidar returns without warning).
    traj_z = traj.all_positions()[:, 2]
    traj_z_min, traj_z_max = float(traj_z.min()), float(traj_z.max())
    if args.min_z is None:
        args.min_z = traj_z_min - args.min_z_margin
    if args.max_z is None:
        args.max_z = traj_z_max + args.max_z_margin
    print(f'[zgate] trajectory Z ∈ [{traj_z_min:+.2f}, {traj_z_max:+.2f}] m  '
          f'→ keeping ENU points with z ∈ [{args.min_z:+.2f}, {args.max_z:+.2f}] m '
          f'(margins: −{args.min_z_margin:.0f}/+{args.max_z_margin:.0f})', flush=True)

    # Trajectory CSV (before lidar pass — useful even if PCD aborts)
    traj_csv = out_dir / 'trajectory_enu.csv'
    with open(traj_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_ns', 'x_enu_m', 'y_enu_m', 'z_enu_m'])
        for p in traj._poses:
            w.writerow([p.t_ns, f'{p.pos[0]:.4f}', f'{p.pos[1]:.4f}', f'{p.pos[2]:.4f}'])
    print(f'[out] trajectory CSV: {traj_csv}', flush=True)

    origin_yaml = out_dir / 'map_origin.yaml'
    with open(origin_yaml, 'w') as f:
        f.write('# AWSIM map origin (ENU local-tangent anchor)\n')
        f.write(f'origin:\n')
        f.write(f'  latitude:  {enu.lat0:.10f}\n')
        f.write(f'  longitude: {enu.lon0:.10f}\n')
        f.write(f'  altitude:  {enu.alt0:.4f}\n')
        f.write(f'frame: ENU\n')
    print(f'[out] map origin YAML: {origin_yaml}', flush=True)

    if not args.skip_lanelet2:
        positions = traj.all_positions()
        ll2_path = out_dir / 'lanelet2_map.osm'
        info = write_lanelet2_osm(str(ll2_path), positions, enu,
                                   half_lane_width=args.half_lane_width,
                                   step_m=args.lanelet_step_m)
        print(f'[out] lanelet2: {ll2_path}  ({info["centerline_points"]} centerline pts)',
              flush=True)

    vmap, counters, ego_dbg = build_pcd(args.mcap, enu, traj, tf_static, args)
    pts = vmap.points()
    pcd_path = out_dir / 'pointcloud_map.pcd'
    write_pcd_binary(str(pcd_path), pts)
    print(f'[out] PCD: {pcd_path}  ({pts.shape[0]:,} voxel-mean points, '
          f'{pcd_path.stat().st_size/1e6:.1f} MB)', flush=True)

    if ego_dbg is not None:
        ego_pts = ego_dbg.points()
        ego_path = out_dir / 'ego_returns.pcd'
        write_pcd_binary(str(ego_path), ego_pts)
        print(f'[out] ego returns (DEBUG): {ego_path}  ({ego_pts.shape[0]:,} pts, '
              f'{ego_path.stat().st_size/1e6:.1f} MB)', flush=True)

    print_filter_report(counters)


if __name__ == '__main__':
    main()
