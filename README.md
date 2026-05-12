# build_AWSIM_map

Tooling that turns a real-world ROS 2 sensor recording into the artefacts a
full **AWSIM + Autoware** simulation needs to load a track for self-driving
testing. Two sets of outputs, both produced by this repo:

**Autoware side** (consumed by `map_loader`, NDT, planner):

- **`pointcloud_map.pcd`** — LiDAR-built point cloud of the environment, used
  by Autoware's NDT matching for localisation.
- **`lanelet2_map.osm`** — vector HD map (centreline + lane boundaries) used
  by the planner and the simulator for routing and behaviour.
- **`map_origin.yaml`** — the geodetic anchor (lat / lon / alt) for the local
  ENU frame the rest of the pipeline lives in.

**AWSIM side** (consumed by Unity for physics + sensor simulation):

- **`meshes/ground.obj`** — drivable surface for the Unity wheel raycast.
- **`meshes/wall_left.obj`**, **`meshes/wall_right.obj`** — generated wall
  candidates from vertical PCD returns. Check `meshes_summary.txt` and inspect
  coverage before relying on them as continuous barriers.
- **`meshes/meshes_manifest.yaml`** — file → Unity layer + RGL material
  category mapping.

The pipeline runs in three stages — pure Python, no ROS or PCL required:

```
data/<track>/raw.db3                            (input: ROS 2 SQLite-3 bag)
                │
                │  db3_to_mcap_converter/convert_db3_to_mcap.py
                │     (one pass: embeds schemas, bakes Whale Dynamic's
                │      sensor_tf_no_camera.yaml into /tf_static, and
                │      renames the secondary lidar's frame_id)
                ▼
data/<track>/cleaned_tf.mcap                    (Foxglove-loadable, TF-aware)
                │
                │  mcap_to_pcd/build_map.py
                │     (pose pass + dual-lidar voxel accumulation +
                │      Lanelet2 OSM seed; auto Z-gate from trajectory)
                ▼
map/<track>/{pointcloud_map.pcd,                (Autoware-side: NDT input)
             lanelet2_map.osm,                  (Autoware-side: lane HD map)
             map_origin.yaml,                   (geodetic anchor)
             trajectory_enu.csv}                (used as road centreline below)
                │
                │  pcd_to_meshes/build_meshes.py
                │     (corridor filter + PCA verticality + ground heightmap +
                │      per-side (arclength, height) wall mesh)
                ▼
map/<track>/meshes/{ground.obj,                 (AWSIM-side: Unity colliders)
                    wall_left.obj,
                    wall_right.obj,
                    meshes_manifest.yaml}       (Unity layer + RGL material)
```

## Repository layout

```
build_AWSIM_map/
├── data/                       Sample sensor recordings (fetched on demand)
│   └── TM99_uphill/
│       ├── download.sh             pulls the source bag from OneDrive
│       ├── raw.db3                 ROS 2 humble bag, SQLite-3 storage      (~89 GiB, gitignored)
│       └── cleaned_tf.mcap         schemas embedded + /tf_static baked in  (~89 GiB, gitignored)
├── db3_to_mcap_converter/      .db3 → .mcap converter (one pass: embeds
│                               canonical ROS 2 schemas, bakes Whale
│                               Dynamic's sensor_tf_no_camera.yaml into
│                               /tf_static, and renames the secondary
│                               lidar's frame_id)
├── mcap_to_pcd/                cleaned_tf.mcap → Autoware-side map
│                               (pose extraction, dual-lidar voxel
│                               accumulation with ego-bbox vehicle-self
│                               filter, Lanelet2 OSM seed)
├── pcd_to_meshes/              pointcloud_map.pcd + trajectory_enu.csv →
│                               AWSIM-side Unity meshes (corridor filter,
│                               PCA verticality classifier, ground
│                               heightmap, per-side wall meshes, OBJ
│                               + layer manifest)
└── map/                        Pre-built AWSIM map outputs (tracked in repo;
    └── TM99_uphill/                pointcloud_map.pcd via Git LFS)
        ├── pointcloud_map.pcd       binary PCD, x/y/z/intensity (LFS-tracked, ~1.8 GiB)
        ├── lanelet2_map.osm         seed Lanelet2 (geodetic lat/lon)
        ├── map_origin.yaml          ENU geodetic anchor
        ├── trajectory_enu.csv       decimated ego trajectory in ENU
        └── meshes/
            ├── ground.obj               drivable surface (Unity layer: Ground)
            ├── wall_left.obj            inner cliff face   (Unity layer: Wall)
            ├── wall_right.obj           outer parapet      (Unity layer: Wall)
            └── meshes_manifest.yaml     mesh → layer + RGL material map
```

Each subdirectory has its own README with the run commands, expected runtime,
and tunables.

## Cloning the repo (Git LFS)

The map's `pointcloud_map.pcd` (~1.8 GiB) and the Unity mesh files
(`*.obj`, ~140 MB for `ground.obj` alone) are tracked via Git LFS, so
make sure LFS is installed before you clone or push:

```bash
git lfs install                           # one-time, per machine
git clone https://github.com/hitchopen/build_AWSIM_map.git
# or, if you already cloned without LFS active:
git lfs pull
```

Without LFS, the PCD and OBJ files come down as tiny pointer files —
Autoware will reject the PCD and Unity will refuse to import the meshes.

## Quick start

If you only want to *use* the pre-built map, you can stop after `git clone +
git lfs pull` — `map/TM99_uphill/` already contains the generated Autoware
artefacts plus Unity OBJ mesh outputs.

To **regenerate** the map from the source bag (or when working with a new
recording), the TM99 sample bag (~180 GiB total) isn't stored in the repo —
fetch it from the sponsor's OneDrive mirror once first:

```bash
bash data/TM99_uphill/download.sh
```

```bash
# 1. one-off: convert the source bag (schemas + /tf_static in one pass)
cd db3_to_mcap_converter
pip install --user -r requirements.txt
python3 convert_db3_to_mcap.py \
    --src ../data/TM99_uphill/raw.db3 \
    --dst ../data/TM99_uphill/cleaned_tf.mcap   # ~60–90 min, ~96 GB output

# 2. build the Autoware-side map (PCD + Lanelet2 + origin + trajectory)
cd ../mcap_to_pcd
pip install --user -r requirements.txt
python3 build_map.py \
    --mcap ../data/TM99_uphill/cleaned_tf.mcap \
    --out-dir ../map/TM99_uphill \
    --voxel-size 0.2 \
    --lidar-decimate 5                          # ~5–10 min

# 3. build the AWSIM-side Unity meshes (ground + walls + manifest)
cd ../pcd_to_meshes
pip install --user -r requirements.txt
python3 build_meshes.py \
    --pcd ../map/TM99_uphill/pointcloud_map.pcd \
    --trajectory ../map/TM99_uphill/trajectory_enu.csv \
    --out-dir ../map/TM99_uphill/meshes         # ~2–5 min
```

After step 2, `map/TM99_uphill/pointcloud_map.pcd` is in local ENU and
`map/TM99_uphill/map_origin.yaml` carries the geodetic anchor — feed
both to Autoware's `map_loader` and the simulator knows where every point
sits in WGS84. `lanelet2_map.osm` is already in geodetic lat/lon natively.

After step 3, `map/TM99_uphill/meshes/` contains Unity-importable geometry:
drop the `.obj` files into your AWSIM scene and assign layers per
`meshes_manifest.yaml` (see "Launching in AWSIM" below). The Unity ENU
origin matches `map_origin.yaml`, so the meshes register exactly against
the PCD without any offset. Before driving, inspect `meshes_summary.txt`
and the OBJs themselves; sparse wall triangle counts mean the generated
walls are not yet suitable as continuous side barriers.

## Launching in AWSIM

[AWSIM](https://github.com/tier4/AWSIM) is the open-source Unity-based
driving simulator from TIER IV. AWSIM provides the simulated vehicle,
sensors, and Unity scene geometry; **Autoware** (running alongside AWSIM
over ROS 2) consumes the map files for localisation, planning, and
visualisation.

A working track for AWSIM + Autoware co-simulation needs **both** sides:

| side | files | consumed by | role |
|---|---|---|---|
| Autoware | `pointcloud_map.pcd`, `lanelet2_map.osm`, `map_origin.yaml` | `map_loader`, NDT, planner | localisation + routing |
| Unity (AWSIM) | `meshes/ground.obj`, `meshes/wall_left.obj`, `meshes/wall_right.obj` | Unity physics + RGL lidar | wheels roll on ground; wall meshes can contain the car and provide lidar surfaces when their generated triangle coverage is dense enough |

This repo produces both. The Unity meshes are aligned to the same ENU
origin as the PCD, so once both sides are loaded with the same
`map_origin.yaml`, the geometry registers exactly.

### 1. Lay the Autoware-side files out the way `map_loader` expects

Pick any directory and drop the three artefacts plus a
`map_projector_info.yaml` next to them:

```
my_track_map/
├── pointcloud_map.pcd                ← copy from map/TM99_uphill/
├── lanelet2_map.osm                  ← copy from map/TM99_uphill/
├── map_projector_info.yaml           ← create — see below
└── (trajectory_enu.csv)              ← optional, for analysis only
```

Autoware reads `map_projector_info.yaml` to know how local-Cartesian PCD
coordinates relate to global lat/lon. Generate it from the values in
`map_origin.yaml`:

```yaml
# map_projector_info.yaml — tells Autoware the map is in a local-tangent
# ENU frame anchored at this geodetic point.
projector_type: local
vertical_datum: WGS84
map_origin:
  latitude:  29.0697514213       # copy from map_origin.yaml
  longitude: 110.4705323434
  altitude:  327.9443
```

For Autoware-style MGRS projection instead of local ENU, set
`projector_type: MGRS` and replace `map_origin` with `mgrs_grid:` plus the
relevant grid string — AWSIM's pre-shipped tutorial maps use this scheme.

### 2. Drop the Unity meshes into the AWSIM scene

In the AWSIM Unity project, create a new empty GameObject (e.g. `TrackEnv`)
and import the three OBJs from `map/<track>/meshes/`:

```
Assets/MyTrack/
├── ground.obj
├── wall_left.obj
├── wall_right.obj
└── meshes_manifest.yaml          ← reference for layer/material assignment
```

Per the manifest:

| OBJ | Unity layer | Unity tag | RGL material | collider |
|---|---|---|---|---|
| `ground.obj` | `Ground` | `Ground` | `asphalt` | MeshCollider |
| `wall_left.obj` | `Wall` | `Wall` | `rock` | MeshCollider |
| `wall_right.obj` | `Wall` | `Wall` | `metal` | MeshCollider |

For each prefab Unity creates from the OBJ:

1. Set the GameObject's **Layer** in the Inspector. AWSIM's vehicle wheel
   raycast mask is set to the `Ground` layer — meshes on any other layer
   will be invisible to the wheels and the car will fall through.
2. Add a **MeshCollider** component (Convex = off — these are environment
   meshes, not physics props).
3. (Optional) Open AWSIM's RGL Mesh Material Properties asset and assign
   the `rgl_material` from the manifest. This shapes the simulated lidar
   intensity returns so they better match the real-bag intensity that
   NDT was tuned against.

Anchor the `TrackEnv` root at Unity world origin `(0, 0, 0)` — the
meshes are already in ENU metres relative to `map_origin.yaml`, so no
transform is needed for them to align with the PCD.

Note on the `Wall` layer: AWSIM's stock tutorial scenes don't ship with
a `Wall` layer; create one in **Edit → Project Settings → Tags and
Layers** (any unused User Layer slot is fine), then add the same name
under **Tags** so the GameObject's Layer + Tag both resolve.

### 3. Get AWSIM and Autoware

- **AWSIM** — clone from <https://github.com/tier4/AWSIM> and follow the
  setup guide at <https://tier4.github.io/AWSIM/>. For a custom track
  (anything other than the tutorial scene) you'll need to open the project
  in Unity to add your `TrackEnv` and rebuild a binary; the pre-built
  binary only ships with TIER IV's reference scene.
- **Autoware** — follow the [Universe install guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/),
  prebuilt Docker images cover most setups.

### 4. Launch the simulator

Run AWSIM and Autoware in two terminals:

```bash
# Terminal 1 — AWSIM (Unity)
./AWSIM.x86_64                                              # or `AWSIM.app` on macOS
```

```bash
# Terminal 2 — Autoware, pointed at this map
ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$(pwd)/my_track_map \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit
```

`e2e_simulator.launch.xml` brings up `map_loader`, `pointcloud_map_loader`,
`lanelet2_map_loader`, NDT localisation, mission/behaviour planning, and the
Rviz visualisation. AWSIM publishes simulated `/sensing/lidar/...` and
`/sensing/imu` topics — RGL casts rays against your `ground.obj` /
`wall_*.obj` meshes to generate them; Autoware locks onto the PCD with NDT,
looks up the ego pose against the Lanelet2, and feeds back planning/control
to the simulator. You can drive a route by setting a goal in Rviz the usual
way.

### 5. Quick visualisation without Autoware

If you just want to look at the map (no planner, no localiser), drag
`pointcloud_map.pcd` and `lanelet2_map.osm` into [Foxglove](https://foxglove.dev/)
or open the OSM in JOSM (with the `lanelet2-plugin`). The OSM carries
geodetic coordinates natively; the PCD and OBJs are local ENU metres and
use `map_origin.yaml` as their global anchor. The `.obj` meshes open in
any viewer (MeshLab, Blender, three.js viewers, even Preview on macOS).

## About the TM99 sample

**TM99** stands for **T**ianmen **M**ountain **99**-Turn — the famously serpentine
hill-climb road carved into the cliffs of Tianmen Mountain in Zhangjiajie,
Hunan, China. The bag in `data/TM99_uphill/` is one full uphill run captured
with an RTK-GPS / SPAN-INS / Hesai-LiDAR / dual-camera stack:

- **31 minutes** of recording, **1.94 M** messages across **17 topics**
- **10.74 km** path, **715 m** elevation gain (327.9 m → 1043.4 m)
- 174 k INSPVAX poses (≈100 Hz, RTK-fixed most of the time)
- 18 k LiDAR scans (Hesai 64-line, 115 k points / scan, ≈3 MB each)
- 18 k H.264-compressed images per camera (front + secondary)
- IMU (raw + corrected) and EKF-fused pose at 100–1000 Hz

The TM99 dataset was **collected and provided by [Whale Dynamic](https://whaledynamic.com/)**,
a sponsor of the **Hitch Open World AI Championships**. It is included here
(under `data/TM99_uphill/`) as a runnable end-to-end sample for the pipeline.

## Credits

This project was designed and is maintained by **Dr. Allen Y. Yang** (Hitch
Interactive · University of California, Berkeley).

Sample dataset (TM99 uphill run): **Whale Dynamic**, contributed in support
of the **Hitch Open World AI Championships**.

## License

This repository, including the sample dataset under `data/TM99_uphill/`, is
licensed under the Creative Commons Attribution-NonCommercial 4.0 International
license (CC BY-NC 4.0). See `LICENSE` for details.

The TM99 sample dataset retains its original attribution to Whale Dynamic and is
included under the same CC BY-NC 4.0 terms for non-commercial use.
