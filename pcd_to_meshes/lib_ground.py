"""Ground mesh via XY heightmap rasterization.

Approach (deliberately simple — robust for an outdoor hill-climb road):

  1. Bin all corridor points into an XY grid of `cell_size` metres.
  2. Per cell, take the LOW PERCENTILE of Z (default 10th). This rejects
     vegetation, signs, parked cars, and ego-vehicle returns that survived
     upstream filtering, and keeps the actual road surface.
  3. Smooth the height field by a small box filter (one pass) to suppress
     per-cell quantisation noise.
  4. Mesh: each cell with coverage becomes two triangles. Cells without
     coverage are skipped — the mesh is naturally trimmed to the road
     corridor instead of being a giant rectangular sheet.
  5. (Optional) Drop tiny disconnected island components — flood-fill on
     the populated-cell mask and keep only components above
     `min_component_cells`. Keeps the largest contiguous road surface.

Output is a triangle mesh in ENU coordinates, ready for OBJ export and
Unity import as a MeshCollider on the `Ground` layer.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class GroundMesh:
    vertices: np.ndarray   # (V, 3) float64, ENU XYZ
    faces: np.ndarray      # (F, 3) int32, triangle indices (CCW from +Z up)
    cell_size: float       # for diagnostics
    coverage_cells: int    # number of populated grid cells used
    grid_shape: Tuple[int, int]  # (nx, ny)


def _percentile_per_cell(values: np.ndarray, cell_idx: np.ndarray,
                         n_cells: int, q: float) -> np.ndarray:
    """Per-cell q-percentile of `values` grouped by `cell_idx` ∈ [0, n_cells).
    Returns an array of length n_cells; cells with no values get NaN.

    Implemented with sort-by-cell-then-take-percentile-per-run; O(N log N).
    """
    order = np.argsort(cell_idx, kind='stable')
    sorted_cells = cell_idx[order]
    sorted_vals  = values[order]
    diff = np.empty(sorted_cells.shape[0], dtype=bool)
    diff[0] = True
    diff[1:] = sorted_cells[1:] != sorted_cells[:-1]
    starts = np.flatnonzero(diff)
    ends = np.append(starts[1:], sorted_cells.shape[0])
    out = np.full(n_cells, np.nan, dtype=np.float64)
    for s, e in zip(starts, ends):
        cid = sorted_cells[s]
        # np.percentile is fine for small per-cell groups; for big ones it's
        # still O(k log k) and k averages a few hundred points per cell at 0.5 m.
        out[cid] = np.percentile(sorted_vals[s:e], q)
    return out


def _box_smooth(grid: np.ndarray, mask: np.ndarray, radius: int) -> np.ndarray:
    """Replace each populated cell's value with the mean over its (2r+1)x(2r+1)
    neighbourhood, considering only populated neighbours. Empty cells stay NaN.
    Vectorised: shift-and-add over (2r+1)^2 neighbour offsets."""
    if radius <= 0:
        return grid
    nx, ny = grid.shape
    pad_v = np.where(mask, grid, 0.0)
    pad_m = mask.astype(np.float64)
    sum_v = np.zeros_like(grid)
    sum_m = np.zeros_like(grid)
    for dx in range(-radius, radius + 1):
        x_dst_lo = max(0, dx);  x_dst_hi = nx + min(0, dx)
        x_src_lo = max(0, -dx); x_src_hi = nx - max(0, dx)
        for dy in range(-radius, radius + 1):
            y_dst_lo = max(0, dy);  y_dst_hi = ny + min(0, dy)
            y_src_lo = max(0, -dy); y_src_hi = ny - max(0, dy)
            sum_v[x_dst_lo:x_dst_hi, y_dst_lo:y_dst_hi] += \
                pad_v[x_src_lo:x_src_hi, y_src_lo:y_src_hi]
            sum_m[x_dst_lo:x_dst_hi, y_dst_lo:y_dst_hi] += \
                pad_m[x_src_lo:x_src_hi, y_src_lo:y_src_hi]
    with np.errstate(invalid='ignore', divide='ignore'):
        smooth = np.where(sum_m > 0, sum_v / np.maximum(sum_m, 1e-9), grid)
    return np.where(mask, smooth, np.nan)


def _enclosed_empty_mask(mask: np.ndarray) -> np.ndarray:
    """Return a boolean array marking the empty cells that form ENCLOSED
    holes in `mask` — i.e. empty cells whose 4-connected component does NOT
    reach the grid boundary. Cells reachable from the boundary through empty
    space are treated as "exterior" (off-road / open air) and excluded.

    This is what makes a hole-fill an actual hole-fill rather than morphological
    dilation: it prevents the road mesh from growing outward into the
    surrounding empty cells while still closing interior gaps."""
    from scipy.ndimage import label
    nx, ny = mask.shape
    empty = ~mask
    structure = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.int32)
    labels, n_comp = label(empty.astype(np.int32), structure=structure)
    if n_comp == 0:
        return np.zeros_like(mask)
    boundary_labels = set()
    boundary_labels.update(np.unique(labels[0,  :]).tolist())
    boundary_labels.update(np.unique(labels[-1, :]).tolist())
    boundary_labels.update(np.unique(labels[:,  0]).tolist())
    boundary_labels.update(np.unique(labels[:, -1]).tolist())
    boundary_labels.discard(0)
    enclosed = empty.copy()
    for lbl in boundary_labels:
        enclosed[labels == lbl] = False
    return enclosed


def _hole_fill_once(grid: np.ndarray, mask: np.ndarray,
                    min_neighbors: int = 3,
                    enclosed_only: bool = True,
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Single-pass hole fill: any unpopulated cell with at least `min_neighbors`
    of its 8 neighbours populated gets filled with the mean Z of those
    neighbours.

    Args:
      grid: (nx, ny) Z values, NaN for unpopulated cells.
      mask: (nx, ny) bool, True for populated cells.
      min_neighbors: 1..8. Lower fills more aggressively. Default 3.
      enclosed_only: if True, only cells that lie in ENCLOSED holes are
                     candidates for filling — empty cells reachable from the
                     grid boundary through empty space are left alone. Use
                     True for ground (avoids dilating road boundaries into
                     off-road areas); use False for walls (where the (s, h)
                     grid's boundaries ARE the wall edges, and we want a thin
                     1-cell curtain to thicken into something triangulable).

    Each pass closes 1 cell off the hole boundary, so 4 passes bridge up to
    ~4-cell-wide gaps."""
    nx, ny = grid.shape
    pad_v = np.where(mask, grid, 0.0)
    pad_m = mask.astype(np.int32)
    sum_v = np.zeros_like(grid)
    cnt_n = np.zeros(grid.shape, dtype=np.int32)
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
    if enclosed_only:
        fill_here &= _enclosed_empty_mask(mask)
    new_grid = grid.copy()
    new_grid[fill_here] = sum_v[fill_here] / np.maximum(cnt_n[fill_here], 1)
    new_mask = mask | fill_here
    return new_grid, new_mask


def _largest_components_mask(mask: np.ndarray, min_cells: int) -> np.ndarray:
    """Flood-fill connected components on the populated-cell mask (4-neighbour),
    keep only components with >= min_cells cells. Returns a boolean mask."""
    from scipy.ndimage import label
    structure = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.int32)
    labels, n = label(mask.astype(np.int32), structure=structure)
    if n == 0:
        return mask
    counts = np.bincount(labels.ravel())
    counts[0] = 0  # background
    keep_labels = np.where(counts >= min_cells)[0]
    keep_set = np.zeros(n + 1, dtype=bool)
    keep_set[keep_labels] = True
    return keep_set[labels]


def build_ground_heightmap(points_xyzi: np.ndarray,
                           cell_size: float = 0.5,
                           low_percentile: float = 10.0,
                           smoothing_radius_cells: int = 1,
                           min_component_cells: int = 200,
                           hole_fill_passes: int = 4,
                           ) -> GroundMesh:
    """Rasterize → percentile-Z → smooth → hole-fill → triangulate.

    Args:
      points_xyzi: (N, 4) float32 from filter_to_corridor (or any subset).
      cell_size: XY rasterization cell, metres. 0.5 is a sensible default
                 for a road surface; smaller = more detail + bigger mesh.
      low_percentile: per-cell Z percentile to take. 10 is conservative
                      against vegetation; raise to 30–50 if your input is
                      already ground-only.
      smoothing_radius_cells: 0 disables; 1 averages over 3×3 neighbours.
      min_component_cells: drop islands smaller than this many cells.
      hole_fill_passes: passes of mean-of-neighbours fill applied to the
                        cell mask after smoothing. Each pass closes 1 cell
                        off the hole boundary; 4 passes (default) bridge
                        gaps up to ~4 cells wide (~2 m at 0.5 m cells)
                        without bleeding into genuinely off-road areas.

    Returns a GroundMesh ready for OBJ export.
    """
    if points_xyzi.shape[0] == 0:
        raise ValueError('build_ground_heightmap: no input points')
    pts = points_xyzi[:, :3].astype(np.float64)

    x_min, y_min = pts[:, 0].min(), pts[:, 1].min()
    x_max, y_max = pts[:, 0].max(), pts[:, 1].max()
    nx = int(np.ceil((x_max - x_min) / cell_size)) + 1
    ny = int(np.ceil((y_max - y_min) / cell_size)) + 1

    ix = np.clip(((pts[:, 0] - x_min) / cell_size).astype(np.int64), 0, nx - 1)
    iy = np.clip(((pts[:, 1] - y_min) / cell_size).astype(np.int64), 0, ny - 1)
    cell_idx = ix * ny + iy
    n_cells = nx * ny

    z_per_cell = _percentile_per_cell(pts[:, 2], cell_idx, n_cells, low_percentile)
    grid = z_per_cell.reshape(nx, ny)
    mask = ~np.isnan(grid)

    # Connectivity: drop little floating cells / patches
    if min_component_cells > 0:
        mask = _largest_components_mask(mask, min_component_cells)
        grid = np.where(mask, grid, np.nan)

    # Smooth
    if smoothing_radius_cells > 0:
        grid = _box_smooth(grid, mask, smoothing_radius_cells)
        mask = ~np.isnan(grid)

    # Hole-fill: bridge small gaps inside the road surface so the strict
    # heightmap triangulator below can produce a watertight collider rather
    # than peppering the road with enclosed empty raster holes.
    for _ in range(max(0, hole_fill_passes)):
        grid, mask = _hole_fill_once(grid, mask, min_neighbors=3)

    # Build vertices (one per populated cell, at cell-centre XY + cell Z)
    cell_to_vid = -np.ones((nx, ny), dtype=np.int64)
    populated = np.argwhere(mask)
    n_v = populated.shape[0]
    if n_v == 0:
        raise RuntimeError('build_ground_heightmap: no populated cells survived filtering')
    cell_to_vid[populated[:, 0], populated[:, 1]] = np.arange(n_v)

    cx = x_min + (populated[:, 0] + 0.5) * cell_size
    cy = y_min + (populated[:, 1] + 0.5) * cell_size
    cz = grid[populated[:, 0], populated[:, 1]]
    vertices = np.column_stack([cx, cy, cz]).astype(np.float64)

    # Triangulate the heightmap permissively: emit two CCW triangles when all
    # 4 quad corners exist, ONE triangle when exactly 3 corners exist (covers
    # the residual holes the hole-fill couldn't reach with `min_neighbors=3`,
    # so the mesh has no enclosed empty raster cells where 3-of-4 corners
    # are populated). Same pattern as lib_walls.py but always CCW from above.
    a = cell_to_vid[:-1, :-1]   # cell (i,   j)
    b = cell_to_vid[1:,  :-1]   # cell (i+1, j)
    c = cell_to_vid[:-1, 1: ]   # cell (i,   j+1)
    d = cell_to_vid[1:,  1: ]   # cell (i+1, j+1)
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

    # Two-triangle quad (CCW from above):
    f_full_1 = np.column_stack([a[full], b[full], d[full]])
    f_full_2 = np.column_stack([a[full], d[full], c[full]])
    # 3-corner cases (CCW):
    f_miss_a = np.column_stack([b[miss_a], d[miss_a], c[miss_a]])
    f_miss_b = np.column_stack([a[miss_b], d[miss_b], c[miss_b]])
    f_miss_c = np.column_stack([a[miss_c], b[miss_c], d[miss_c]])
    f_miss_d = np.column_stack([a[miss_d], b[miss_d], c[miss_d]])

    faces = np.concatenate(
        [f_full_1, f_full_2, f_miss_a, f_miss_b, f_miss_c, f_miss_d],
        axis=0,
    ).astype(np.int32)

    return GroundMesh(
        vertices=vertices,
        faces=faces,
        cell_size=cell_size,
        coverage_cells=int(mask.sum()),
        grid_shape=(nx, ny),
    )
