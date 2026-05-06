#!/usr/bin/env python3
"""Build an AWSIM track map (PCD + Lanelet2 + map_origin) from a cleaned MCAP bag.

Two passes over the bag:

  1. Pose pass — read every /pp7/inspvax message, convert lat/lon/alt to local
     ENU and (roll, pitch, azimuth) to a rotation that maps a point in the
     LiDAR frame straight into ENU world. Filters out PROPOGATED fixes by
     default (they're INS dead-reckoned without RTK).

  2. LiDAR pass — for each (decimated) /lidar_points_compressed scan, look up
     the nearest pose by timestamp, transform points into ENU, voxel-downsample,
     accumulate. The aggregator keeps per-voxel sums + counts so the final
     mean is exact regardless of intermediate flushes.

Outputs (under --out-dir):
    pointcloud_map.pcd      binary PCD (x,y,z,intensity float32)
    lanelet2_map.osm        seed Lanelet2: centerline + ±half-width boundaries
    map_origin.yaml         lat/lon/alt of ENU origin (== first INS fix unless overridden)
    trajectory_enu.csv      decimated ego trajectory in ENU (for sanity-check plots)

Run:
    python3 build_map.py \
        --mcap ../data/TM99_uphill/cleaned.mcap \
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

# Local libs
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib_pose import ENUFrame, Pose, TrajectoryStore, lidar_to_enu_rotation
from lib_pcd import StreamingVoxelMap, write_pcd_binary
from lib_lanelet2 import write_lanelet2_osm

from mcap.reader import make_reader
from mcap_ros2._dynamic import generate_dynamic


INSPVAX_TYPE = 'starneto_gps_msgs/msg/Inspvax'
NAVSATFIX_TYPE = 'sensor_msgs/msg/NavSatFix'
POINTCLOUD2_TYPE = 'sensor_msgs/msg/PointCloud2'

LIDAR_TOPIC = '/lidar_points_compressed'
INSPVAX_TOPIC = '/pp7/inspvax'


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--mcap', required=True, help='input MCAP bag')
    p.add_argument('--out-dir', required=True, help='where to write PCD + OSM + yaml')

    # Origin
    p.add_argument('--origin-lat', type=float, default=None,
                   help='ENU origin latitude (deg). Default: first usable INS fix.')
    p.add_argument('--origin-lon', type=float, default=None,
                   help='ENU origin longitude (deg). Default: first usable INS fix.')
    p.add_argument('--origin-alt', type=float, default=None,
                   help='ENU origin altitude (m, ellipsoidal). Default: first usable INS fix.')

    # PCD
    p.add_argument('--voxel-size', type=float, default=0.2, help='PCD voxel grid (m). Default 0.2')
    p.add_argument('--lidar-topic', default=LIDAR_TOPIC)
    p.add_argument('--lidar-decimate', type=int, default=5,
                   help='Use every Nth lidar scan. Default 5 (i.e. 2 Hz @ 10 Hz source).')
    p.add_argument('--min-range', type=float, default=2.0,
                   help='drop points closer than this many metres from sensor (filters ego)')
    p.add_argument('--max-range', type=float, default=120.0,
                   help='drop points farther than this many metres from sensor')
    p.add_argument('--min-z', type=float, default=-5.0,
                   help='drop transformed points below origin by this much (in ENU). Default -5.')
    p.add_argument('--max-z', type=float, default=300.0,
                   help='drop transformed points above origin by this much. Default 300.')

    # Trajectory filtering
    p.add_argument('--accept-propagated', action='store_true',
                   help='accept INS_PROPOGATED fixes (lower quality). Default: only keep RTK-class.')
    p.add_argument('--max-pose-dt-ms', type=int, default=50,
                   help='LiDAR scan rejected if no pose within this many ms. Default 50.')

    # Lanelet2
    p.add_argument('--half-lane-width', type=float, default=1.75)
    p.add_argument('--lanelet-step-m', type=float, default=5.0)
    p.add_argument('--skip-lanelet2', action='store_true')

    # Limits / debug
    p.add_argument('--max-scans', type=int, default=None,
                   help='Hard cap on lidar scans processed (for debugging).')
    return p.parse_args()


# ---------- pose pass ----------

ACCEPTABLE_POSITION_TYPES_RTK = {
    'INS_RTKFIXED', 'INS_RTKFLOAT', 'INS_PSRDIFF', 'INS_PSRSP',
    'NARROW_INT', 'NARROW_FLOAT', 'L1_FLOAT', 'WIDE_INT', 'IONOFREE_FLOAT',
}


def extract_trajectory(mcap_path: str, accept_propagated: bool,
                       origin_override) -> tuple[ENUFrame, TrajectoryStore]:
    """Pass 1: read every /pp7/inspvax, build (origin, trajectory)."""
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
        n_kept = 0
        for sch2, ch, msg in reader.iter_messages(topics=[INSPVAX_TOPIC]):
            n_seen += 1
            d = decoder(msg.data)
            ptype = d.position_type
            if not accept_propagated and ptype not in ACCEPTABLE_POSITION_TYPES_RTK:
                continue
            raw_traj.append((msg.log_time, d.latitude, d.longitude, d.altitude,
                             d.roll, d.pitch, d.azimuth))
            n_kept += 1
            now = time.time()
            if now - last_print > 5.0:
                print(f'  [pose] {n_seen}/{n_total} read, {n_kept} kept   '
                      f'rate {n_seen/(now-t0):.0f} msg/s', flush=True)
                last_print = now

    if not raw_traj:
        raise SystemExit('no usable INSPVAX poses found (try --accept-propagated)')

    print(f'[pose] kept {len(raw_traj)} of {n_seen} INSPVAX poses '
          f'({100.0*len(raw_traj)/max(n_seen,1):.1f}%) in {time.time()-t0:.1f}s', flush=True)

    # Build ENU origin
    if origin_override[0] is not None:
        lat0, lon0, alt0 = origin_override
        print(f'[pose] using user-specified origin: lat={lat0:.7f} lon={lon0:.7f} alt={alt0:.2f}',
              flush=True)
    else:
        _, lat0, lon0, alt0, *_ = raw_traj[0]
        print(f'[pose] using first usable INS fix as origin: '
              f'lat={lat0:.7f} lon={lon0:.7f} alt={alt0:.2f}', flush=True)
    enu = ENUFrame(lat0, lon0, alt0)

    # Build trajectory store
    store = TrajectoryStore()
    for t_ns, lat, lon, alt, roll, pitch, az in raw_traj:
        pos = enu.to_enu(lat, lon, alt)
        R = lidar_to_enu_rotation(roll, pitch, az)
        store.append(Pose(t_ns=t_ns, pos=pos, R_lidar_to_world=R))

    print(f'[pose] built trajectory store: {len(store)} poses', flush=True)
    return enu, store


# ---------- lidar pass ----------

def parse_pointcloud2(d) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Decode a sensor_msgs/PointCloud2 with fields x,y,z (f32), intensity (f32),
    ring (u16), timestamp (f64). Returns (x, y, z, intensity) as float32 arrays."""
    width = d.width
    height = d.height
    n = width * height
    step = d.point_step
    if n == 0 or step == 0:
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty, empty
    buf = bytes(d.data)
    arr = np.frombuffer(buf, dtype=np.uint8)[: n * step].reshape(n, step)
    xs = np.frombuffer(arr[:, 0:4].tobytes(), dtype=np.float32)
    ys = np.frombuffer(arr[:, 4:8].tobytes(), dtype=np.float32)
    zs = np.frombuffer(arr[:, 8:12].tobytes(), dtype=np.float32)
    ii = np.frombuffer(arr[:, 12:16].tobytes(), dtype=np.float32)
    return xs, ys, zs, ii


def build_pcd(mcap_path: str,
              enu: ENUFrame,
              traj: TrajectoryStore,
              args) -> StreamingVoxelMap:
    """Pass 2: stream lidar -> voxel-downsampled accumulator."""
    print(f'[pcd] streaming {args.lidar_topic} (decimate={args.lidar_decimate}, '
          f'voxel={args.voxel_size}m)', flush=True)
    vmap = StreamingVoxelMap(voxel_size=args.voxel_size)

    max_dt_ns = int(args.max_pose_dt_ms * 1e6)
    min_r2 = args.min_range ** 2
    max_r2 = args.max_range ** 2

    n_processed = 0
    n_skipped_no_pose = 0
    n_emitted_pts = 0
    t0 = time.time()
    last_print = t0

    with open(mcap_path, 'rb') as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        sch = next(s for s in summary.schemas.values() if s.name == POINTCLOUD2_TYPE)
        decoder = generate_dynamic(sch.name, sch.data.decode())[sch.name]
        n_total = sum(c for cid, c in summary.statistics.channel_message_counts.items()
                      if summary.channels[cid].topic == args.lidar_topic)

        idx = -1
        for sch2, ch, msg in reader.iter_messages(topics=[args.lidar_topic]):
            idx += 1
            if idx % args.lidar_decimate != 0:
                continue
            if args.max_scans is not None and n_processed >= args.max_scans:
                break

            pose = traj.lookup(msg.log_time, max_dt_ns=max_dt_ns)
            if pose is None:
                n_skipped_no_pose += 1
                continue

            d = decoder(msg.data)
            xs, ys, zs, ii = parse_pointcloud2(d)
            if xs.size == 0:
                continue

            # Range filter on raw lidar points (in lidar frame)
            r2 = xs * xs + ys * ys + zs * zs
            mask = (r2 >= min_r2) & (r2 <= max_r2) & np.isfinite(r2)
            if not mask.any():
                continue
            xs, ys, zs, ii = xs[mask], ys[mask], zs[mask], ii[mask]

            # Transform: world = R @ local + t
            local = np.column_stack([xs, ys, zs]).astype(np.float64)
            world = (pose.R_lidar_to_world @ local.T).T + pose.pos  # (N, 3)
            # Z gate in ENU
            zmask = (world[:, 2] >= args.min_z) & (world[:, 2] <= args.max_z)
            if not zmask.any():
                continue
            world = world[zmask]
            ii = ii[zmask]

            xyzi = np.column_stack([world.astype(np.float32),
                                     ii.astype(np.float32, copy=False)])
            vmap.add(xyzi)

            n_processed += 1
            n_emitted_pts += xyzi.shape[0]

            now = time.time()
            if now - last_print > 5.0:
                rate = n_processed / max(now - t0, 0.001)
                pct = 100.0 * (idx + 1) / max(n_total, 1)
                stats = vmap.stats()
                print(f'  [pcd] scans {n_processed:6d} ({pct:5.1f}% of bag)  '
                      f'rate {rate:.1f} sc/s  voxels {stats["voxels"]:>8,}  '
                      f'pending {stats["pending"]:>9,}', flush=True)
                last_print = now

    print(f'[pcd] processed {n_processed} scans in {time.time()-t0:.1f}s '
          f'({n_skipped_no_pose} skipped, no pose)  emitted {n_emitted_pts:,} raw points',
          flush=True)
    return vmap


# ---------- main ----------

def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    origin = (args.origin_lat, args.origin_lon, args.origin_alt)
    if any(v is None for v in origin) and not all(v is None for v in origin):
        raise SystemExit('--origin-lat/-lon/-alt must all be supplied or all be omitted')

    enu, traj = extract_trajectory(args.mcap, args.accept_propagated, origin)

    # Save trajectory CSV (before lidar pass — useful even if PCD aborts)
    traj_csv = out_dir / 'trajectory_enu.csv'
    with open(traj_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_ns', 'x_enu_m', 'y_enu_m', 'z_enu_m'])
        for p in traj._poses:
            w.writerow([p.t_ns, f'{p.pos[0]:.4f}', f'{p.pos[1]:.4f}', f'{p.pos[2]:.4f}'])
    print(f'[out] trajectory CSV: {traj_csv}', flush=True)

    # Map origin yaml
    origin_yaml = out_dir / 'map_origin.yaml'
    with open(origin_yaml, 'w') as f:
        f.write('# AWSIM map origin (ENU local-tangent anchor)\n')
        f.write(f'origin:\n')
        f.write(f'  latitude:  {enu.lat0:.10f}\n')
        f.write(f'  longitude: {enu.lon0:.10f}\n')
        f.write(f'  altitude:  {enu.alt0:.4f}\n')
        f.write(f'frame: ENU\n')
    print(f'[out] map origin YAML: {origin_yaml}', flush=True)

    # Lanelet2
    if not args.skip_lanelet2:
        positions = traj.all_positions()
        ll2_path = out_dir / 'lanelet2_map.osm'
        info = write_lanelet2_osm(str(ll2_path), positions, enu,
                                   half_lane_width=args.half_lane_width,
                                   step_m=args.lanelet_step_m)
        print(f'[out] lanelet2: {ll2_path}  ({info["centerline_points"]} centerline points, '
              f'lane width {2*info["half_lane_width_m"]} m)', flush=True)

    # PCD pass
    vmap = build_pcd(args.mcap, enu, traj, args)
    pts = vmap.points()
    pcd_path = out_dir / 'pointcloud_map.pcd'
    write_pcd_binary(str(pcd_path), pts)
    print(f'[out] PCD: {pcd_path}  ({pts.shape[0]:,} voxel-mean points, '
          f'{pcd_path.stat().st_size/1e6:.1f} MB)', flush=True)


if __name__ == '__main__':
    main()
