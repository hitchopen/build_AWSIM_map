# mcap → AWSIM track map

Builds the three artefacts an AWSIM/Autoware simulation needs to load a track,
straight from `data/<track>/cleaned.mcap`:

```
pointcloud_map.pcd      binary PCD (x, y, z, intensity, float32) — for NDT / scenery
lanelet2_map.osm        seed Lanelet2 — centerline + ±1.75 m lane boundaries
map_origin.yaml         lat/lon/alt of the local ENU origin
trajectory_enu.csv      decimated ego trajectory (debug aid)
```

This is a **seed map**, not a finished HD map. Centerline + lane is exactly the
ego trajectory — fine for a test track / hillclimb, but you'll want to refine it
in [Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder/) or
JOSM (with the lanelet2 plugin) before treating it as production.

## What it does

Two passes over the bag:

1. **Pose pass** — read every `/pp7/inspvax`, drop INS_PROPOGATED fixes by
   default (RTK-only), convert WGS84 → local ENU and Novatel
   (roll, pitch, azimuth) → a 3×3 rotation that takes a point in the LiDAR
   frame straight into ENU world. Coordinate frames involved:
   - WGS84:  geodetic
   - **ENU**: X east, Y north, Z up (the simulation world frame)
   - NED:    SPAN local-level (X north, Y east, Z down)
   - body (FRD): SPAN body — X forward, Y right, Z down
   - lidar (FLU): Hesai sensor — X forward, Y left, Z up
2. **LiDAR pass** — for each (decimated) `/lidar_points_compressed` scan, look
   up the nearest pose by timestamp, range-filter the points, transform them
   into ENU, and feed them into a streaming voxel-grid accumulator. The
   accumulator keeps per-voxel sums + counts so the final per-voxel mean is
   exact regardless of when the buffer is flushed.

LiDAR-to-IMU translation is treated as zero (no `/tf_static` in the bag). On
this dataset that introduces ~0.5–1.5 m vertical bias — fine for a track-following
simulation, less so for tight localization. If you have a calibrated mount
offset, apply it to the points by editing `lib_pose.py:lidar_to_enu_rotation`.

## Install dependencies

```bash
pip install --user numpy mcap mcap-ros2-support
```

Python ≥ 3.10. No ROS or PCL needed.

## Run

```bash
cd /Users/yang/Documents/GitHub/build_AWSIM_map/mcap_to_SIM

python3 build_map.py \
    --mcap ../data/TM99_uphill/cleaned.mcap \
    --out-dir ../map/TM99_uphill \
    --voxel-size 0.2 \
    --lidar-decimate 5
```

Expected runtime: ~5–10 min on a modern laptop. The pose pass dominates if you
keep all 174 k INSPVAX messages; the LiDAR pass is ~3 min at decimate=5.

Estimated outputs for the full TM99 hillclimb bag (4.7 km, 680 m elevation gain):
- PCD: ≈ 200–500 MB at 0.2 m voxel (depends on tree/wall density)
- Lanelet2: ≈ 1 k centerline points at 5 m step
- trajectory CSV: ≈ 100 k rows

## Choosing the map origin

By default the origin is the **first usable INS fix** (the first
`/pp7/inspvax` with an RTK-class position type). To pin it explicitly, pass
all three:

```bash
python3 build_map.py ... \
    --origin-lat 29.0697514 \
    --origin-lon 110.4705323 \
    --origin-alt 327.944
```

The chosen origin is recorded in `map_origin.yaml`. Lanelet2 nodes are written
in geodetic coordinates so any Autoware projector (UTM, MGRS, local ENU) can
load the same OSM file.

## Useful flags

| flag | purpose |
| --- | --- |
| `--voxel-size` | PCD grid resolution. 0.2 m balances detail vs. file size. 0.5 m for a quicker build. |
| `--lidar-decimate` | Keep every Nth scan (default 5 → 2 Hz from 10 Hz). Lower = denser map, longer build. |
| `--min-range / --max-range` | Drop points outside this radius from the sensor (sensor-frame). Default 2–120 m. |
| `--min-z / --max-z` | Drop points below/above this ENU height. Default −5 to +300 m. |
| `--accept-propagated` | Allow INS_PROPOGATED fixes (no RTK lock). Default off. |
| `--max-pose-dt-ms` | Reject a scan if no pose within this many ms. Default 50. |
| `--half-lane-width` | Lateral offset for the lanelet's left/right boundaries (m). |
| `--lanelet-step-m` | Decimate the centerline by arc-length to this step. |
| `--skip-lanelet2` | Skip the OSM file (PCD + origin only). |
| `--max-scans` | Hard cap on lidar scans processed (for debugging). |

## Files

```
build_map.py          orchestrator + CLI
lib_pose.py           ENU frame, geodetic↔ECEF, body→NED→ENU rotations
lib_pcd.py            streaming voxel accumulator + binary PCD writer
lib_lanelet2.py       centerline → boundaries → Lanelet2 OSM
```

## Loading in AWSIM

Drop the three deliverables into your AWSIM map config:

```
my_map/
├── pointcloud_map.pcd
├── lanelet2_map.osm
└── map_projector_info.yaml   # set projector_type: local, origin from map_origin.yaml
```

For Autoware-style MGRS projection, convert the lat/lon in `map_origin.yaml`
to MGRS (e.g. with `mgrspy` or `geographiclib`'s `MGRS` class) and set
`projector_type: MGRS` instead.

## Refining the seed Lanelet2

The seed map gives you a single one-way lane along the ego trajectory. Real
tracks often want:
- multiple lanelets for different segments,
- explicit start/finish lines (`type=stop_line`),
- regulatory elements for traffic lights / pit boxes,
- corrected lane widths where the road actually narrows.

Open `lanelet2_map.osm` in JOSM (with `lanelet2-plugin`) or
TIER IV Vector Map Builder, refine by hand, and save back to the same
filename — Autoware will pick it up.
