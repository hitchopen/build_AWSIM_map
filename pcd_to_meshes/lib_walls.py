"""Wall mesh extraction from corridor points.

The job is to turn the cliff face on one side of the road and the parapet /
guardrail / drop-off on the other side into two separate triangle meshes
that AWSIM can use as colliders + lidar-return surfaces.

Two-step approach, both pure numpy + scipy.spatial.cKDTree:

  1. Verticality classification — for each point, compute the local PCA
     over its k-nearest neighbours. The smallest eigenvector is the
     surface normal estimate; if its Z component magnitude is small
     (|n_z| < vertical_thresh), the local surface is roughly vertical
     → it's a wall, not ground.

  2. Side classification — points already carry a signed lateral offset
     from `filter_to_corridor`. Sign > 0 = left of travel; < 0 = right.
     We split the wall points into two clouds and mesh each independently.

Meshing per side uses a "(s, h) heightmap" trick that avoids needing a
full 3D mesher:

    s = arclength along the trajectory (from `corridor.along`)
    h = height above the local trajectory Z

  Bin (s, h) into a 2D grid; per cell, take the median lateral offset
  (signed_lat) — that's the wall's actual transverse position at that
  (arclength, height). This collapses an inherently 2.5D structure
  (a curving curtain) into a regular grid we can mesh as quads, then
  re-embed each (s, h) cell back into 3D ENU using the trajectory's
  XY position and tangent at arclength s.

The result is a smooth, watertight-along-arc curtain following the road,
with one mesh for the left wall and one for the right.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


# ---------- verticality classifier --------------------------------------------

def classify_vertical(points_xyz: np.ndarray,
                      k: int = 16,
                      vertical_thresh: float = 0.4,
                      ) -> np.ndarray:
    """Per-point boolean mask: True = locally vertical surface.

    Args:
      points_xyz: (N, 3) float64.
      k: neighbours for local PCA. 12–20 is fine; bigger k is smoother
         but costs more.
      vertical_thresh: |normal_z| below this counts as vertical.
                       0.4 ≈ surfaces tilted within ~66° of vertical.

    Method: for each point, fetch its k nearest neighbours (3D Euclidean),
    centre, compute covariance, take the eigenvector for the smallest
    eigenvalue as the surface normal estimate. We avoid a Python per-point
    loop by batching the eigh: covariances are (N, 3, 3) and numpy.linalg
    handles the broadcast.
    """
    from scipy.spatial import cKDTree
    if points_xyz.shape[0] == 0:
        return np.empty(0, dtype=bool)

    pts = points_xyz.astype(np.float64)
    n = pts.shape[0]
    kdt = cKDTree(pts)

    # Process in chunks to bound peak memory: each chunk allocates ~ chunk*k*3 doubles.
    chunk = 200_000
    out = np.empty(n, dtype=bool)
    for start in range(0, n, chunk):
        end = min(start + chunk, n)
        q = pts[start:end]
        dists, idx = kdt.query(q, k=k + 1)  # +1 because the first hit is self
        idx = idx[:, 1:]                    # drop self
        nb = pts[idx]                       # (chunk, k, 3)
        centred = nb - nb.mean(axis=1, keepdims=True)
        # Covariance per point: (chunk, 3, 3)
        cov = np.einsum('ckm,ckn->cmn', centred, centred) / max(k - 1, 1)
        # Symmetric eigendecomposition (ascending eigenvalues)
        w, v = np.linalg.eigh(cov)
        normals = v[:, :, 0]                # eigenvector for smallest eigval
        out[start:end] = np.abs(normals[:, 2]) < vertical_thresh
    return out


# ---------- per-side wall mesher ----------------------------------------------

@dataclass
class WallMesh:
    vertices: np.ndarray   # (V, 3) float64
    faces: np.ndarray      # (F, 3) int32
    side: str              # 'left' or 'right'
    arclength_cells: int
    height_cells: int


def _trajectory_basis(trajectory: np.ndarray
                      ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per-trajectory-point (arclength, unit XY tangent, unit XY left-normal)."""
    seg = np.linalg.norm(np.diff(trajectory[:, :2], axis=0), axis=1)
    arclen = np.concatenate([[0.0], np.cumsum(seg)])
    tan = np.zeros_like(trajectory[:, :2])
    tan[1:-1] = trajectory[2:, :2] - trajectory[:-2, :2]
    tan[0]    = trajectory[1, :2]  - trajectory[0, :2]
    tan[-1]   = trajectory[-1, :2] - trajectory[-2, :2]
    nrm = np.linalg.norm(tan, axis=1, keepdims=True)
    nrm[nrm == 0] = 1.0
    tan = tan / nrm
    # Left normal in 2D = rotate tangent +90°: (tx, ty) -> (-ty, tx)
    left = np.column_stack([-tan[:, 1], tan[:, 0]])
    return arclen, tan, left


def _hole_fill_once(lat_grid: np.ndarray, mask: np.ndarray,
                    min_neighbors: int = 3) -> tuple[np.ndarray, np.ndarray]:
    """Single-pass thicken/fill: any unpopulated cell with at least
    `min_neighbors` of its 8 neighbours populated gets filled with the mean
    lateral offset of those neighbours.

    Unlike the ground variant in lib_ground.py, this does NOT restrict to
    enclosed empty cells. Cliff returns from a tilted lidar are naturally
    striped in (arclength, height) and the wall mesh frequently starts as
    1-cell-wide chains where every empty cell is connected to the (s, h)
    grid boundary (open air above/below the cliff). Restricting to enclosed
    cells would leave those chains 1-wide and prevent any 4-corner quads
    from forming. The mild dilation here is intentional and necessary."""
    nx, ny = lat_grid.shape
    pad_v = np.where(mask, lat_grid, 0.0)
    pad_m = mask.astype(np.int32)
    sum_v = np.zeros_like(lat_grid)
    cnt_n = np.zeros(lat_grid.shape, dtype=np.int32)
    for dx in (-1, 0, 1):
        x_dst_lo = max(0, dx);  x_dst_hi = nx + min(0, dx)
        x_src_lo = max(0, -dx); x_src_hi = nx - max(0, dx)
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            y_dst_lo = max(0, dy);  y_dst_hi = ny + min(0, dy)
            y_src_lo = max(0, -dy); y_src_hi = ny - max(0, dy)
            sum_v[x_dst_lo:x_dst_hi, y_dst_lo:y_dst_hi] += \
                pad_v[x_src_lo:x_src_hi, y_src_lo:y_src_hi]
            cnt_n[x_dst_lo:x_dst_hi, y_dst_lo:y_dst_hi] += \
                pad_m[x_src_lo:x_src_hi, y_src_lo:y_src_hi]
    fill_here = (~mask) & (cnt_n >= min_neighbors)
    new_lat = lat_grid.copy()
    new_lat[fill_here] = sum_v[fill_here] / np.maximum(cnt_n[fill_here], 1)
    new_mask = mask | fill_here
    return new_lat, new_mask


def _percentile_per_cell(values: np.ndarray, cell_idx: np.ndarray,
                         n_cells: int, q: float) -> np.ndarray:
    """Same helper as in lib_ground — duplicated here to keep modules independent."""
    if cell_idx.size == 0:
        return np.full(n_cells, np.nan, dtype=np.float64)
    order = np.argsort(cell_idx, kind='stable')
    sc = cell_idx[order]
    sv = values[order]
    diff = np.empty(sc.shape[0], dtype=bool)
    diff[0] = True
    diff[1:] = sc[1:] != sc[:-1]
    starts = np.flatnonzero(diff)
    ends = np.append(starts[1:], sc.shape[0])
    out = np.full(n_cells, np.nan, dtype=np.float64)
    for s, e in zip(starts, ends):
        out[sc[s]] = np.percentile(sv[s:e], q)
    return out


def build_wall_mesh(points_xyzi: np.ndarray,
                    along: np.ndarray,           # (N,) arclength of each pt
                    signed_lat: np.ndarray,      # (N,) lateral offset (left=+)
                    nearest_idx: np.ndarray,     # (N,) traj index per point
                    trajectory: np.ndarray,
                    side: str,                   # 'left' or 'right'
                    arclength_cell: float = 1.0,
                    height_cell: float = 1.0,
                    max_height: float = 30.0,
                    min_component_cells: int = 50,
                    hole_fill_passes: int = 2,
                    max_edge_m: float = 5.0,
                    ) -> WallMesh:
    """Build a wall mesh on one side of the trajectory.

    Args:
      points_xyzi: (N, 4) float32 — already vertical-classified points.
      along, signed_lat, nearest_idx: from CorridorResult.
      trajectory: (T, 3) ENU XYZ — the polyline the road follows.
      side: 'left' picks signed_lat > 0; 'right' picks < 0.
      arclength_cell: bin size along the road (m). 1.0 captures decent detail
                      on switchbacks; raise to 2–5 m for big files.
      height_cell: bin size in vertical (m). 1.0 is the default — anything smaller
                   makes the (s, h) heightmap too sparse for stable triangulation
                   on long road sections (cliff returns are naturally striped in
                   height). Lower to 0.5 only if you've tuned hole_fill_passes up.
      max_height: clip wall above this many metres (cliffs can be tall;
                  the lidar only ever sees the lower portion anyway).
      min_component_cells: drop dust islands of fewer cells than this.
      hole_fill_passes: passes of mean-of-neighbours fill applied to the (s, h)
                        mask after the component filter. 2 passes bridge gaps
                        of up to ~2 cells. Set 0 to disable.
      max_edge_m: post-triangulation guard. Drop any triangle whose longest
                  3D edge in ENU exceeds this many metres. The (s, h) grid
                  treats arclength as continuous, but if the trajectory has
                  a GPS dropout / sharp switchback / large decimator gap,
                  two adjacent (s, h) cells can map to physically distant
                  ENU positions and form a long bridge triangle that acts
                  as an unwanted collider / lidar-return surface. The
                  default 5 m is ~5× the default arclength_cell — outlier
                  bridges across switchbacks tend to be 60–110 m, so they
                  are easily caught. Set <= 0 to disable.
    """
    if side not in ('left', 'right'):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    # Side mask
    if side == 'left':
        side_mask = signed_lat > 0
    else:
        side_mask = signed_lat < 0
    if not side_mask.any():
        raise RuntimeError(f'no wall points on side={side}')
    pts = points_xyzi[side_mask]
    along_s = along[side_mask].astype(np.float64)
    lat_s   = np.abs(signed_lat[side_mask]).astype(np.float64)
    nidx_s  = nearest_idx[side_mask]

    # Height above local road Z
    z_road = trajectory[nidx_s, 2]
    h = pts[:, 2].astype(np.float64) - z_road

    # Discard everything below road or above max_height
    keep = (h >= -2.0) & (h <= max_height)
    if not keep.any():
        raise RuntimeError(f'no wall points within height band on side={side}')
    along_s, lat_s, h = along_s[keep], lat_s[keep], h[keep]
    pts = pts[keep]
    nidx_s = nidx_s[keep]

    s_min = along_s.min()
    s_max = along_s.max()
    h_min = max(-2.0, h.min())
    h_max = min(max_height, h.max())

    n_s = int(np.ceil((s_max - s_min) / arclength_cell)) + 1
    n_h = int(np.ceil((h_max - h_min) / height_cell)) + 1
    is_ = np.clip(((along_s - s_min) / arclength_cell).astype(np.int64), 0, n_s - 1)
    ih_ = np.clip(((h        - h_min) / height_cell   ).astype(np.int64), 0, n_h - 1)
    cell_idx = is_ * n_h + ih_

    # Per-cell median lateral offset (robust to a stray point or two)
    lat_grid = _percentile_per_cell(lat_s, cell_idx, n_s * n_h, q=50.0).reshape(n_s, n_h)

    mask = ~np.isnan(lat_grid)
    if mask.sum() == 0:
        raise RuntimeError(f'wall grid empty on side={side}')

    # Hole fill / thicken FIRST, component filter SECOND. Order matters:
    #
    # Cliff returns are naturally striped in (arclength, height) — at any
    # one arclength only a few discrete heights actually get lidar hits.
    # Initially the populated mask is often 1-cell-wide chains that ARE
    # spatially connected (so a component filter at min=50 keeps them) but
    # CAN'T form 4-corner quads (so the triangulator produces almost nothing
    # — the bug the reviewer caught with 98 / 44 triangles).
    #
    # By running hole-fill first we thicken those chains into 2-3-cell-wide
    # bands which then triangulate cleanly. Doing it after the component
    # filter wouldn't help — the chains would already be the only structure.
    #
    # Pass 1 uses min_neighbors=2 to reach into 1-wide chains (a cell
    # adjacent to a 1-wide chain has 2 chain neighbours horizontally,
    # 1 vertically). Subsequent passes use min_neighbors=3 for stability.
    if hole_fill_passes >= 1:
        lat_grid, mask = _hole_fill_once(lat_grid, mask, min_neighbors=2)
    for _ in range(max(0, hole_fill_passes - 1)):
        lat_grid, mask = _hole_fill_once(lat_grid, mask, min_neighbors=3)

    # Drop tiny components AFTER thickening (curtain noise far from the road).
    if min_component_cells > 0:
        from scipy.ndimage import label
        structure = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.int32)
        labels, ncomp = label(mask.astype(np.int32), structure=structure)
        if ncomp > 0:
            counts = np.bincount(labels.ravel())
            counts[0] = 0
            keep_lbls = np.where(counts >= min_component_cells)[0]
            keep_set = np.zeros(ncomp + 1, dtype=bool)
            keep_set[keep_lbls] = True
            mask = keep_set[labels]
            lat_grid = np.where(mask, lat_grid, np.nan)

    # Re-embed each populated cell into ENU using the trajectory basis at s
    arclen_traj, tan_traj, left_norm_traj = _trajectory_basis(trajectory)
    # cell-centre arclength
    s_centres = s_min + (np.arange(n_s) + 0.5) * arclength_cell
    # Find each s_centre's nearest traj index by interpolation
    traj_idx_per_s = np.searchsorted(arclen_traj, s_centres).clip(0, trajectory.shape[0] - 1)
    traj_xy_per_s   = trajectory[traj_idx_per_s, :2]
    z_road_per_s    = trajectory[traj_idx_per_s, 2]
    left_per_s      = left_norm_traj[traj_idx_per_s]

    cell_to_vid = -np.ones((n_s, n_h), dtype=np.int64)
    populated = np.argwhere(mask)
    n_v = populated.shape[0]
    if n_v == 0:
        raise RuntimeError(f'wall mesh has no surviving cells on side={side}')
    cell_to_vid[populated[:, 0], populated[:, 1]] = np.arange(n_v)

    # Build vertices
    s_idx = populated[:, 0]
    h_idx = populated[:, 1]
    lat   = lat_grid[s_idx, h_idx]                          # (V,)
    sign  = +1.0 if side == 'left' else -1.0
    # XY = traj_xy + sign * lat * left_normal
    xy_v  = traj_xy_per_s[s_idx] + (sign * lat[:, None]) * left_per_s[s_idx]
    h_v   = h_min + (h_idx + 0.5) * height_cell
    z_v   = z_road_per_s[s_idx] + h_v
    vertices = np.column_stack([xy_v[:, 0], xy_v[:, 1], z_v])

    # Triangulate the (s, h) grid permissively: emit two triangles when all 4
    # quad corners are populated, ONE triangle when exactly 3 are populated
    # (covering the natural sparsity of cliff-return heightmaps). Winding is
    # CW in (s, h) for the LEFT side and CCW for the RIGHT side, so each
    # wall's outward normal faces away from the road in world coordinates.
    a = cell_to_vid[:-1, :-1]   # cell (s,   h)
    b = cell_to_vid[1:,  :-1]   # cell (s+1, h)
    c = cell_to_vid[:-1, 1: ]   # cell (s,   h+1)
    d = cell_to_vid[1:,  1: ]   # cell (s+1, h+1)
    m_a = a >= 0
    m_b = b >= 0
    m_c = c >= 0
    m_d = d >= 0
    n_corners = m_a.astype(np.int8) + m_b.astype(np.int8) + m_c.astype(np.int8) + m_d.astype(np.int8)

    full = n_corners == 4
    only3 = n_corners == 3
    miss_a = only3 & ~m_a   # have b, c, d
    miss_b = only3 & ~m_b   # have a, c, d
    miss_c = only3 & ~m_c   # have a, b, d
    miss_d = only3 & ~m_d   # have a, b, c

    if side == 'left':
        # Two-triangle quad (CW in (s, h)):
        f_full_1 = np.column_stack([a[full], c[full], d[full]])
        f_full_2 = np.column_stack([a[full], d[full], b[full]])
        # 3-corner cases (CW):
        f_miss_a = np.column_stack([b[miss_a], c[miss_a], d[miss_a]])
        f_miss_b = np.column_stack([a[miss_b], c[miss_b], d[miss_b]])
        f_miss_c = np.column_stack([a[miss_c], d[miss_c], b[miss_c]])
        f_miss_d = np.column_stack([a[miss_d], c[miss_d], b[miss_d]])
    else:
        # Two-triangle quad (CCW in (s, h)):
        f_full_1 = np.column_stack([a[full], b[full], d[full]])
        f_full_2 = np.column_stack([a[full], d[full], c[full]])
        # 3-corner cases (CCW = reverse of LEFT winding):
        f_miss_a = np.column_stack([b[miss_a], d[miss_a], c[miss_a]])
        f_miss_b = np.column_stack([a[miss_b], d[miss_b], c[miss_b]])
        f_miss_c = np.column_stack([a[miss_c], b[miss_c], d[miss_c]])
        f_miss_d = np.column_stack([a[miss_d], b[miss_d], c[miss_d]])

    faces = np.concatenate(
        [f_full_1, f_full_2, f_miss_a, f_miss_b, f_miss_c, f_miss_d],
        axis=0,
    ).astype(np.int32)

    # Bridge-triangle filter: drop any face whose longest 3D edge in ENU
    # exceeds max_edge_m. The (s, h) grid is monotonic in arclength, but
    # if the trajectory has a GPS dropout or a sharp switchback compresses
    # several cells worth of arclength into a tiny ENU step, two adjacent
    # cells can map to physically distant world positions and form a long
    # bridge triangle that acts as an unwanted collider / lidar surface.
    if max_edge_m and max_edge_m > 0 and faces.shape[0] > 0:
        v0 = vertices[faces[:, 0]]
        v1 = vertices[faces[:, 1]]
        v2 = vertices[faces[:, 2]]
        e01 = np.linalg.norm(v1 - v0, axis=1)
        e12 = np.linalg.norm(v2 - v1, axis=1)
        e20 = np.linalg.norm(v0 - v2, axis=1)
        max_edge = np.maximum(np.maximum(e01, e12), e20)
        keep = max_edge <= max_edge_m
        n_dropped = int((~keep).sum())
        if n_dropped > 0:
            faces = faces[keep]
        # NB: we deliberately don't garbage-collect now-orphaned vertices —
        # OBJ allows unused vertices, and Unity / collider import ignores
        # them. Keeping the index arithmetic simple is worth the few KB.

    return WallMesh(
        vertices=vertices,
        faces=faces,
        side=side,
        arclength_cells=n_s,
        height_cells=n_h,
    )
