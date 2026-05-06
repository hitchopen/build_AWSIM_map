"""Streaming voxel-grid accumulator + binary PCD writer.

Memory model: as new (x,y,z,intensity) points come in, we buffer them. When
the buffer exceeds a threshold we collapse new + previously-summarised voxels
into one consolidated table indexed by packed integer voxel keys. Per-voxel
we keep (sum_x, sum_y, sum_z, sum_intensity, count) so re-merging across
flushes stays exact: the final per-voxel mean is sum/count regardless of how
many flushes happened in between.
"""
from __future__ import annotations

import os
import struct
from typing import Tuple

import numpy as np


def _pack_keys(int_keys: np.ndarray) -> np.ndarray:
    """Pack 3 int64 voxel indices into a single int64 (21 bits each, signed)."""
    mask = np.int64((1 << 21) - 1)
    return ((int_keys[:, 0] & mask) << 42) | \
           ((int_keys[:, 1] & mask) << 21) | \
           (int_keys[:, 2] & mask)


def _aggregate(keys: np.ndarray, sums: np.ndarray, counts: np.ndarray
               ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Group rows by `keys`, summing `sums` and `counts`. Returns ascending by key."""
    if keys.shape[0] == 0:
        return keys, sums, counts
    order = np.argsort(keys, kind='stable')
    sk = keys[order]
    s_sums = sums[order]
    s_counts = counts[order]
    diff = np.empty(sk.shape[0], dtype=bool)
    diff[0] = True
    diff[1:] = sk[1:] != sk[:-1]
    starts = np.flatnonzero(diff)
    unique_keys = sk[starts]
    agg_sums = np.add.reduceat(s_sums, starts, axis=0)
    agg_counts = np.add.reduceat(s_counts, starts)
    return unique_keys, agg_sums, agg_counts


class StreamingVoxelMap:
    """Append (x,y,z,intensity) points and downsample on the fly to a fixed voxel grid."""

    def __init__(self, voxel_size: float, flush_threshold: int = 20_000_000):
        if voxel_size <= 0:
            raise ValueError('voxel_size must be > 0')
        self.voxel_size = float(voxel_size)
        self.flush_threshold = int(flush_threshold)

        self._buf: list[np.ndarray] = []
        self._buf_n = 0

        self._summary_keys = np.empty(0, dtype=np.int64)
        self._summary_sums = np.empty((0, 4), dtype=np.float64)
        self._summary_counts = np.empty(0, dtype=np.int64)

    def add(self, pts_xyzi: np.ndarray) -> None:
        """Add an (N, 4) array of float32 points: x, y, z, intensity. World/ENU coords."""
        if pts_xyzi.size == 0:
            return
        if pts_xyzi.dtype != np.float32:
            pts_xyzi = pts_xyzi.astype(np.float32, copy=False)
        self._buf.append(pts_xyzi)
        self._buf_n += pts_xyzi.shape[0]
        if self._buf_n >= self.flush_threshold:
            self._flush()

    def _flush(self) -> None:
        if not self._buf:
            return
        new = np.concatenate(self._buf, axis=0)
        self._buf = []
        self._buf_n = 0

        int_keys = np.floor(new[:, :3] / self.voxel_size).astype(np.int64)
        new_keys = _pack_keys(int_keys)
        new_sums = new.astype(np.float64)  # 4-col: x, y, z, intensity
        new_counts = np.ones(new.shape[0], dtype=np.int64)
        new_keys, new_sums, new_counts = _aggregate(new_keys, new_sums, new_counts)

        if self._summary_keys.shape[0] == 0:
            self._summary_keys = new_keys
            self._summary_sums = new_sums
            self._summary_counts = new_counts
        else:
            keys = np.concatenate([self._summary_keys, new_keys])
            sums = np.concatenate([self._summary_sums, new_sums], axis=0)
            counts = np.concatenate([self._summary_counts, new_counts])
            self._summary_keys, self._summary_sums, self._summary_counts = \
                _aggregate(keys, sums, counts)

    def points(self) -> np.ndarray:
        """Return the (M, 4) per-voxel mean cloud. Forces a final flush."""
        self._flush()
        if self._summary_keys.shape[0] == 0:
            return np.empty((0, 4), dtype=np.float32)
        means = (self._summary_sums / self._summary_counts[:, None]).astype(np.float32)
        return means

    def stats(self) -> dict:
        return {
            'voxels': int(self._summary_keys.shape[0]),
            'pending': self._buf_n,
            'voxel_size': self.voxel_size,
        }


def write_pcd_binary(path: str, points_xyzi: np.ndarray) -> None:
    """Write a binary PCD file with fields x y z intensity (float32) — Autoware-friendly."""
    if points_xyzi.dtype != np.float32:
        points_xyzi = points_xyzi.astype(np.float32, copy=False)
    n = points_xyzi.shape[0]
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA binary\n'
    ).encode('ascii')
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    with open(path, 'wb') as f:
        f.write(header)
        f.write(points_xyzi.tobytes(order='C'))
