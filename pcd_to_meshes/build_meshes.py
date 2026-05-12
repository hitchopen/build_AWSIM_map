#!/usr/bin/env python3
"""Build AWSIM-importable ground + wall meshes from the mcap_to_pcd outputs.

Inputs (produced upstream by mcap_to_pcd/build_map.py):
    pointcloud_map.pcd   binary PCD (x, y, z, intensity float32) in local ENU
    trajectory_enu.csv   decimated ego trajectory in local ENU

Outputs (under --out-dir):
    ground.obj               drivable surface (Unity layer: Ground)
    wall_left.obj            inner cliff face / inboard wall (Unity layer: Wall)
    wall_right.obj           outer parapet / cliff drop (Unity layer: Wall)
    meshes_manifest.yaml     mapping of each .obj to Unity layer + RGL material
    meshes_summary.txt       per-mesh point/triangle counts (debug aid)

Pipeline:
  1. Read PCD + trajectory.
  2. Filter PCD to a corridor around the trajectory (configurable half-width).
  3. Classify each corridor point as locally vertical via PCA on k-NN.
  4. Ground = non-vertical points → XY heightmap → triangle mesh.
  5. Walls  = vertical points     → split left/right of travel → per-side
              (arclength, height) heightmap → triangle mesh.
  6. Write OBJs + YAML manifest + summary.

Run:
    python3 build_meshes.py \\
        --pcd ../map/TM99_uphill/pointcloud_map.pcd \\
        --trajectory ../map/TM99_uphill/trajectory_enu.csv \\
        --out-dir ../map/TM99_uphill/meshes
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib_pcd_io import (
    read_pcd_xyzi, read_trajectory_csv, decimate_trajectory_arclength,
    filter_to_corridor,
)
from lib_ground import build_ground_heightmap
from lib_walls import classify_vertical, build_wall_mesh
from lib_obj import write_obj, estimate_vertex_normals


# ---------- args --------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--pcd', required=True, help='input pointcloud_map.pcd (binary, xyzi)')
    p.add_argument('--trajectory', required=True, help='input trajectory_enu.csv')
    p.add_argument('--out-dir', required=True, help='where to write the .obj + manifest')

    # Trajectory pre-processing
    p.add_argument('--traj-decimate-m', type=float, default=2.0,
                   help='Decimate trajectory so consecutive samples are >= this far apart (m). '
                        'Default 2.0 — drops 100 Hz INS duplicates without losing road shape.')

    # Corridor
    p.add_argument('--corridor-half-width', type=float, default=12.0,
                   help='Keep points within this many metres of the trajectory in XY. '
                        'Default 12 — covers the road + ~10 m of cliff face / parapet.')
    p.add_argument('--corridor-max-height', type=float, default=30.0,
                   help='Max metres ABOVE the local road Z to keep. Default 30. '
                        'Bigger = taller cliff captured; runtime grows roughly linearly.')
    p.add_argument('--corridor-max-depth', type=float, default=10.0,
                   help='Max metres BELOW the local road Z to keep. Default 10. '
                        'Useful on the downhill side where the drop is the wall.')

    # Verticality classifier
    p.add_argument('--vertical-knn', type=int, default=16,
                   help='k-NN size for local PCA. Default 16.')
    p.add_argument('--vertical-thresh', type=float, default=0.4,
                   help='|normal_z| below this is classified as vertical (= wall). '
                        'Default 0.4 (~surfaces tilted within 66° of vertical).')

    # Ground heightmap
    p.add_argument('--ground-cell', type=float, default=0.5,
                   help='XY cell size (m) for ground heightmap. Default 0.5.')
    p.add_argument('--ground-low-percentile', type=float, default=10.0,
                   help='Per-cell Z percentile to take as the ground. Default 10.')
    p.add_argument('--ground-smooth', type=int, default=1,
                   help='Box-smoothing radius in cells for the ground height field. 0 disables.')
    p.add_argument('--ground-min-component-cells', type=int, default=200,
                   help='Drop ground islands smaller than this many cells. Default 200.')
    p.add_argument('--ground-hole-fill-passes', type=int, default=4,
                   help='Hole-fill passes on the ground heightmap mask before '
                        'triangulation. Default 4 — bridges holes up to ~4 cells '
                        '(~2 m at the default 0.5 m cells). 0 disables.')

    # Wall heightmaps
    p.add_argument('--wall-arclength-cell', type=float, default=1.0,
                   help='Arclength bin (m) for wall meshes. Default 1.0.')
    p.add_argument('--wall-height-cell', type=float, default=1.0,
                   help='Vertical bin (m) for wall meshes. Default 1.0 — anything '
                        'smaller makes the (s, h) heightmap too sparse for the '
                        'triangulator to bridge gaps. Lower with care.')
    p.add_argument('--wall-max-height', type=float, default=30.0,
                   help='Clip wall above this many m above the road. Default 30.')
    p.add_argument('--wall-min-component-cells', type=int, default=50,
                   help='Drop wall islands smaller than this many cells. Default 50.')
    p.add_argument('--wall-hole-fill-passes', type=int, default=2,
                   help='Hole-fill passes on the (s, h) wall mask before '
                        'triangulation. Default 2. 0 disables.')
    p.add_argument('--wall-max-edge-m', type=float, default=5.0,
                   help='Drop any wall triangle whose longest 3D edge in ENU '
                        'exceeds this many metres. Catches outlier "bridge" '
                        'triangles spanning trajectory discontinuities '
                        '(switchback inner-radius, GPS dropouts). Default 5 m. '
                        'Set <= 0 to disable.')

    # Misc
    p.add_argument('--no-normals', action='store_true',
                   help='Skip per-vertex normals in the OBJ (smaller files, Unity will generate).')
    p.add_argument('--max-points', type=int, default=None,
                   help='Hard cap on points read from the PCD (debug aid).')
    return p.parse_args()


# ---------- pipeline ----------------------------------------------------------

def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    t0 = time.time()
    print(f'[in ] reading PCD: {args.pcd}', flush=True)
    pts = read_pcd_xyzi(args.pcd)
    if args.max_points is not None and pts.shape[0] > args.max_points:
        # Random subsample (reproducible)
        rng = np.random.default_rng(0)
        sel = rng.choice(pts.shape[0], size=args.max_points, replace=False)
        pts = pts[sel]
    print(f'      → {pts.shape[0]:,} points  '
          f'X∈[{pts[:,0].min():.1f}, {pts[:,0].max():.1f}]  '
          f'Y∈[{pts[:,1].min():.1f}, {pts[:,1].max():.1f}]  '
          f'Z∈[{pts[:,2].min():.1f}, {pts[:,2].max():.1f}]', flush=True)

    print(f'[in ] reading trajectory: {args.trajectory}', flush=True)
    traj = read_trajectory_csv(args.trajectory)
    print(f'      → {traj.shape[0]:,} samples  (decimating to >= {args.traj_decimate_m} m steps)',
          flush=True)
    traj = decimate_trajectory_arclength(traj, step_m=args.traj_decimate_m)
    print(f'      → {traj.shape[0]:,} samples after decimation', flush=True)

    # ----- corridor filter
    print(f'[corr] filtering points to corridor (half-width={args.corridor_half_width} m)…',
          flush=True)
    cor = filter_to_corridor(pts, traj,
                             half_width_m=args.corridor_half_width,
                             max_height_above_road=args.corridor_max_height,
                             max_depth_below_road=args.corridor_max_depth)
    print(f'      → kept {cor.points.shape[0]:,} of {pts.shape[0]:,} '
          f'({100.0*cor.points.shape[0]/max(pts.shape[0],1):.1f}%)', flush=True)
    if cor.points.shape[0] == 0:
        raise SystemExit('corridor filter rejected all points')

    # Free the full-cloud array — we don't need it any more
    del pts

    # ----- verticality
    print(f'[clas] PCA verticality (k={args.vertical_knn}, thresh={args.vertical_thresh})…',
          flush=True)
    vert_mask = classify_vertical(cor.points[:, :3].astype(np.float64),
                                   k=args.vertical_knn,
                                   vertical_thresh=args.vertical_thresh)
    n_vert = int(vert_mask.sum())
    n_grnd = int((~vert_mask).sum())
    print(f'      → vertical (walls): {n_vert:,}   non-vertical (ground): {n_grnd:,}',
          flush=True)

    summary = {}

    # ----- ground
    print(f'[grnd] heightmap (cell={args.ground_cell} m, p{args.ground_low_percentile:.0f})…',
          flush=True)
    ground_pts = cor.points[~vert_mask]
    ground = build_ground_heightmap(
        ground_pts,
        cell_size=args.ground_cell,
        low_percentile=args.ground_low_percentile,
        smoothing_radius_cells=args.ground_smooth,
        min_component_cells=args.ground_min_component_cells,
        hole_fill_passes=args.ground_hole_fill_passes,
    )
    print(f'      → {ground.vertices.shape[0]:,} verts, {ground.faces.shape[0]:,} tris '
          f'({ground.coverage_cells:,} populated cells of {ground.grid_shape[0]*ground.grid_shape[1]:,})',
          flush=True)
    g_normals = None if args.no_normals else estimate_vertex_normals(ground.vertices, ground.faces)
    g_path = out_dir / 'ground.obj'
    write_obj(str(g_path), ground.vertices, ground.faces, normals=g_normals,
              object_name='Ground')
    print(f'[out ] {g_path}  ({g_path.stat().st_size/1e6:.1f} MB)', flush=True)
    summary['ground.obj'] = {
        'vertices': int(ground.vertices.shape[0]),
        'triangles': int(ground.faces.shape[0]),
        'cell_size_m': float(ground.cell_size),
    }

    # ----- walls (left + right)
    wall_pts        = cor.points[vert_mask]
    wall_along      = cor.along[vert_mask]
    wall_signed_lat = cor.signed_lat[vert_mask]
    wall_nidx       = cor.nearest_idx[vert_mask]

    for side in ('left', 'right'):
        print(f'[wall] {side} (arc={args.wall_arclength_cell} m, h={args.wall_height_cell} m)…',
              flush=True)
        w_path = out_dir / f'wall_{side}.obj'
        try:
            wall = build_wall_mesh(
                wall_pts, wall_along, wall_signed_lat, wall_nidx, traj,
                side=side,
                arclength_cell=args.wall_arclength_cell,
                height_cell=args.wall_height_cell,
                max_height=args.wall_max_height,
                min_component_cells=args.wall_min_component_cells,
                hole_fill_passes=args.wall_hole_fill_passes,
                max_edge_m=args.wall_max_edge_m,
            )
        except RuntimeError as e:
            # Remove any stale wall_<side>.obj from a previous run so the
            # output directory only contains files this run produced.
            print(f'      [skip] {e}', flush=True)
            if w_path.exists():
                w_path.unlink()
                print(f'      [skip] removed stale {w_path}', flush=True)
            continue
        print(f'      → {wall.vertices.shape[0]:,} verts, {wall.faces.shape[0]:,} tris '
              f'(grid {wall.arclength_cells}×{wall.height_cells})', flush=True)
        w_normals = (None if args.no_normals
                     else estimate_vertex_normals(wall.vertices, wall.faces))
        w_path = out_dir / f'wall_{side}.obj'
        write_obj(str(w_path), wall.vertices, wall.faces, normals=w_normals,
                  object_name=f'Wall_{side.capitalize()}')
        print(f'[out ] {w_path}  ({w_path.stat().st_size/1e6:.1f} MB)', flush=True)
        summary[f'wall_{side}.obj'] = {
            'vertices': int(wall.vertices.shape[0]),
            'triangles': int(wall.faces.shape[0]),
            'arclength_cell_m': float(args.wall_arclength_cell),
            'height_cell_m': float(args.wall_height_cell),
        }

    # ----- manifest
    manifest = {
        'frame': 'ENU',
        'note': ('Drop these meshes into a Unity scene with the listed layer + '
                 'MeshCollider. RGL material categories are advisory hints for '
                 'AWSIM\'s lidar simulation — set in the RGL Mesh Material '
                 'Properties asset.'),
        'meshes': [],
    }
    if 'ground.obj' in summary:
        manifest['meshes'].append({
            'file': 'ground.obj',
            'unity_layer': 'Ground',
            'unity_tag': 'Ground',
            'rgl_material': 'asphalt',
            'collider': 'MeshCollider',
            'static': True,
        })
    if 'wall_left.obj' in summary:
        manifest['meshes'].append({
            'file': 'wall_left.obj',
            'unity_layer': 'Wall',
            'unity_tag': 'Wall',
            'rgl_material': 'rock',  # inner side: cliff face
            'collider': 'MeshCollider',
            'static': True,
        })
    if 'wall_right.obj' in summary:
        manifest['meshes'].append({
            'file': 'wall_right.obj',
            'unity_layer': 'Wall',
            'unity_tag': 'Wall',
            'rgl_material': 'metal',  # outer side: typically guardrail / parapet
            'collider': 'MeshCollider',
            'static': True,
        })

    manifest_path = out_dir / 'meshes_manifest.yaml'
    with open(manifest_path, 'w') as f:
        yaml.safe_dump(manifest, f, sort_keys=False)
    print(f'[out ] {manifest_path}', flush=True)

    # Summary text
    summary_path = out_dir / 'meshes_summary.txt'
    with open(summary_path, 'w') as f:
        f.write(f'pcd_to_meshes summary\n')
        f.write(f'=====================\n')
        f.write(f'input PCD       : {args.pcd}\n')
        f.write(f'input trajectory: {args.trajectory}\n')
        f.write(f'corridor half-w : {args.corridor_half_width} m\n')
        f.write(f'vertical thresh : |n_z| < {args.vertical_thresh} (k={args.vertical_knn})\n')
        f.write(f'\n')
        for name, info in summary.items():
            f.write(f'{name}\n')
            for k, v in info.items():
                f.write(f'  {k}: {v}\n')
            f.write(f'\n')
    print(f'[out ] {summary_path}', flush=True)

    print(f'\n[done] total {time.time()-t0:.1f}s', flush=True)


if __name__ == '__main__':
    main()
