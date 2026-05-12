# MCAP → Autoware PCD + Lanelet2 map

Builds the Autoware-side map artefacts from the cleaned-and-TF-injected
MCAP that [`db3_to_mcap_converter/convert_db3_to_mcap.py`](../db3_to_mcap_converter/)
produces:

```
pointcloud_map.pcd      binary PCD (x, y, z, intensity, float32) — for NDT / scenery
lanelet2_map.osm        seed Lanelet2 — centerline + ±1.75 m lane boundaries
map_origin.yaml         lat / lon / alt of the local ENU origin
trajectory_enu.csv      decimated ego trajectory in ENU (debug aid)
```

The Lanelet2 is a **seed map**, not a finished HD map: the centerline + lane
is exactly the ego trajectory — fine for a hillclimb / time-trial track, but
you'll want to refine it in
[Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder/) or
JOSM (with the lanelet2 plugin) before treating it as production.

## Pipeline at a glance

```
data/<track>/raw.db3
    │
    │  db3_to_mcap_converter/convert_db3_to_mcap.py
    ▼
data/<track>/cleaned_tf.mcap        ← /tf_static baked in, secondary lidar renamed
    │
    │  mcap_to_pcd/build_map.py     ← THIS DIRECTORY
    ▼
map/<track>/{pointcloud_map.pcd,
             lanelet2_map.osm,
             map_origin.yaml,
             trajectory_enu.csv}
```

`build_map.py` reads `/tf_static` directly from the bag — no separate YAML
needed at this stage. If you convert with `--no-tf` or otherwise omit
`/tf_static`, `build_map.py` will refuse to start with a clear error.

## What the build does

Two passes over the bag:

1. **Pose pass** — read every `/pp7/inspvax`, drop INS_PROPOGATED fixes by
   default (RTK-class only), convert WGS84 → local ENU and (roll, pitch,
   azimuth) → a 3×3 rotation that takes a vector in the bag's `imu` frame
   (FLU body) into ENU world. Coordinate frames involved:
   - WGS84:    geodetic
   - **ENU**:  X east, Y north, Z up — the simulation world frame
   - `imu`:    bag's IMU frame, FLU body — root of the static TF tree
   - lidar:    each lidar's own sensor frame — joined to `imu` via /tf_static
   - NED / FRD: SPAN's native conventions; INSPVAX angles live here, the
                pose pass does the conversion to FLU/ENU.

2. **LiDAR pass** — for each (decimated) scan from each lidar topic:
   1. range-filter raw points in the lidar's own frame,
   2. apply the static `lidar → imu` transform from `/tf_static`,
   3. drop points inside the **ego bounding box** (in IMU/FLU frame),
   4. apply the dynamic `R_imu→ENU(t)` and IMU's ENU position,
   5. voxel-downsample on the fly.

Per-stage point counts are reported per topic at the end so you can see
exactly how many returns each filter removed.

## Ground-truth GNSS coordinates in the output

All three deliverables carry global lat/lon/alt — packaged the standard
AWSIM / Autoware way:

| file | how the global coords are stored | who reads it |
|---|---|---|
| `map_origin.yaml`     | the geodetic anchor itself: `latitude / longitude / altitude` | every Autoware projector |
| `lanelet2_map.osm`    | OSM `<node lat="..." lon="..."/>` natively, plus `<tag k="ele">` for altitude | lanelet2_map_loader |
| `pointcloud_map.pcd`  | local ENU XYZ for efficiency; recover global via `inverse(ENU→ECEF) ∘ ECEF→geodetic` anchored at `map_origin` | NDT, occupancy grid, scenery |

This is the convention `map_loader` expects: feed it the PCD plus a
`map_projector_info.yaml` that points at `map_origin`, and the simulator
knows where every point sits in WGS84.

## Install dependencies

```bash
pip install --user -r requirements.txt
```

Python ≥ 3.10. No ROS or PCL needed.

## Run

```bash
cd mcap_to_pcd                      # from the repo root

python3 build_map.py \
    --mcap ../data/TM99_uphill/cleaned_tf.mcap \
    --out-dir ../map/TM99_uphill \
    --voxel-size 0.2 \
    --lidar-decimate 5
```

Expected runtime: **5–10 min** on a modern laptop for the full TM99 bag
(94 Hz INSPVAX × 1860 s + 2 Hz lidar × 1860 s × 2 streams). Current TM99
output is about **1.8 GiB** at 0.2 m voxels because the Z gate keeps the
full 715 m climb plus cliff margin.

To force a specific map origin (instead of the first INS fix):

```bash
python3 build_map.py ... \
    --origin-lat 29.0697514 \
    --origin-lon 110.4705323 \
    --origin-alt 327.944
```

To debug-write the points the ego-bbox filter rejected (visualisable
side-by-side with `pointcloud_map.pcd`):

```bash
python3 build_map.py ... --save-ego-debug-pcd
```

This produces an extra `ego_returns.pcd` with only the points that fell
inside the ego bounding box (in IMU frame, then transformed into ENU world
for visualisation). It's the easiest way to confirm the bbox is cutting
vehicle structure rather than environment.

## Useful flags

| flag | purpose |
| --- | --- |
| `--voxel-size`        | PCD grid resolution. 0.2 m balances detail vs. file size. 0.5 m for a quicker build. |
| `--lidar-topics`      | Comma-separated PointCloud2 topics to fuse. Default fuses both Hesai units from cleaned_tf.mcap. |
| `--lidar-decimate`    | Use every Nth scan per lidar. Default 5 (≈2 Hz from 10 Hz source). |
| `--min-range / --max-range` | Drop points outside this radius from the sensor (lidar frame). Default 2 m … 120 m. |
| `--ego-bbox`          | "x_min,x_max,y_min,y_max,z_min,z_max" in IMU/FLU metres. Default `-3,+3,-1.2,+1.2,-1.8,+0.5`. Pass `none` to disable. |
| `--save-ego-debug-pcd`| Also write `ego_returns.pcd` containing only the bbox-rejected points (debug aid). |
| `--min-z / --max-z`   | Override the auto-computed ENU altitude band (absolute m). By default the band is derived from the trajectory's actual altitude range (`traj_z_min − 10` to `traj_z_max + 100`). The previous fixed `+300 m` cap silently truncated tracks with > ~280 m elevation gain (e.g. TM99 climbs 715 m) — see commit history. |
| `--min-z-margin / --max-z-margin` | Margin below / above the trajectory altitude range to keep when auto-computing the Z gate. Defaults `10 m` / `100 m` (+100 captures cliff face above the road). |
| `--accept-propagated` | Allow INS_PROPOGATED fixes (no RTK lock). Default off. |
| `--max-pose-dt-ms`    | Reject a scan if no pose within this many ms. Default 50. |
| `--half-lane-width`   | Lateral offset for the lanelet's left/right boundaries (m). Default 1.75. |
| `--lanelet-step-m`    | Decimate the centerline by arc-length to this step. Default 5. |
| `--skip-lanelet2`     | Skip the OSM file (PCD + origin only). |
| `--max-scans`         | Hard cap on lidar scans (per topic). For debugging only. |

## Vehicle-self filter — what it does

Vehicle returns (chassis, hood, mirrors, antennas) are removed by **two
complementary filters** working together:

1. **Range filter** (lidar frame) — drops points within `--min-range` of
   the sensor. Default 2 m — covers most chassis returns because both
   Hesai units are mounted close to the vehicle structure.
2. **Ego bounding box** (IMU/FLU frame, after the static lidar→imu
   transform) — drops points inside an axis-aligned box around the IMU.
   Shape-aware — catches anything that slips through the range gate
   (e.g. a roof rack 2.5 m from a roof-mounted lidar).

The filters are independent — you can disable either by setting
`--min-range 0.0` or `--ego-bbox none`. The default thresholds are tuned
for a typical hillclimb car ~5 m × 2 m × 1.5 m with the IMU mounted
roughly mid-cabin. Adjust `--ego-bbox` to your vehicle's actual dimensions
if needed.

## Sensor calibration

Sensor extrinsics live in
[`db3_to_mcap_converter/sensor_tf_no_camera.yaml`](../db3_to_mcap_converter/sensor_tf_no_camera.yaml)
(official transforms supplied by Whale Dynamic). They're baked into the
bag by `convert_db3_to_mcap.py`; `build_map.py` reads `/tf_static` from
the bag at runtime, so it picks up whatever calibration was injected.

## Files

```
build_map.py             orchestrator + CLI for the Autoware-side map build
lib_pose.py              ENU frame, geodetic↔ECEF, imu→ENU rotation,
                          /tf_static loader
lib_pcd.py               streaming voxel accumulator + binary PCD writer
lib_lanelet2.py          centerline → boundaries → Lanelet2 OSM
requirements.txt
```

## Loading in Autoware

Drop the deliverables into your Autoware map directory:

```
my_map/
├── pointcloud_map.pcd
├── lanelet2_map.osm
└── map_projector_info.yaml   # projector_type: local, origin from map_origin.yaml
```

For Autoware-style MGRS projection, convert the lat/lon in `map_origin.yaml`
to MGRS (e.g. with `mgrspy` or `geographiclib`'s `MGRS` class) and set
`projector_type: MGRS` in `map_projector_info.yaml` instead.

## Refining the seed Lanelet2

The seed gives you a single one-way lane along the ego trajectory. Real
tracks often want:

- multiple lanelets for different segments,
- explicit start / finish lines (`type=stop_line`),
- regulatory elements for traffic lights, pit boxes,
- corrected lane widths where the road actually narrows.

Open `lanelet2_map.osm` in JOSM (with the `lanelet2-plugin`) or in
TIER IV Vector Map Builder, refine by hand, and save back to the same
filename — Autoware will pick it up.
