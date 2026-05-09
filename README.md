# build_AWSIM_map

Tooling that turns a real-world ROS 2 sensor recording into the three artefacts
AWSIM (and Autoware behind it) needs to load a track for self-driving simulation:

- **`pointcloud_map.pcd`** — LiDAR-built point cloud of the environment, used
  by Autoware's NDT matching for localisation.
- **`lanelet2_map.osm`** — vector HD map (centreline + lane boundaries) used
  by the planner and the simulator for routing and behaviour.
- **`map_origin.yaml`** — the geodetic anchor (lat / lon / alt) for the local
  ENU frame the rest of the pipeline lives in.

The pipeline runs in two stages — pure-Python, no ROS or PCL required:

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
                │  mcap_to_SIM/build_map.py
                ▼
map/<track>/{pointcloud_map.pcd,
             lanelet2_map.osm,
             map_origin.yaml,
             trajectory_enu.csv}                (AWSIM-ready map, GNSS-anchored)
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
├── mcap_to_SIM/                cleaned_tf.mcap → AWSIM map (pose extraction,
│                               dual-lidar voxel accumulation with ego-bbox
│                               vehicle-self filter, Lanelet2 OSM seed)
└── map/                        Pre-built AWSIM map outputs (tracked in repo;
    └── TM99_uphill/                pointcloud_map.pcd via Git LFS)
        ├── pointcloud_map.pcd       binary PCD, x/y/z/intensity (LFS-tracked, ~855 MB)
        ├── lanelet2_map.osm         seed Lanelet2 (geodetic lat/lon)
        ├── map_origin.yaml          ENU geodetic anchor
        └── trajectory_enu.csv       decimated ego trajectory in ENU
```

Each subdirectory has its own README with the run commands, expected runtime,
and tunables.

## Cloning the repo (Git LFS)

The map's `pointcloud_map.pcd` is ~855 MB and is tracked via Git LFS, so
make sure LFS is installed before you clone or push:

```bash
git lfs install                           # one-time, per machine
git clone https://github.com/hitchopen/build_AWSIM_map.git
# or, if you already cloned without LFS active:
git lfs pull
```

Without LFS, the PCD comes down as a tiny pointer file — Autoware will
reject it.

## Quick start

If you only want to *use* the pre-built map, you can stop after `git clone +
git lfs pull` — `map/TM99_uphill/` already contains the four AWSIM-ready
artefacts.

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

# 2. build the AWSIM map
cd ../mcap_to_SIM
pip install --user -r requirements.txt
python3 build_map.py \
    --mcap ../data/TM99_uphill/cleaned_tf.mcap \
    --out-dir ../map/TM99_uphill \
    --voxel-size 0.2 \
    --lidar-decimate 5                          # ~5–10 min
```

After step 2 finishes, `map/TM99_uphill/pointcloud_map.pcd` is in local ENU
and `map/TM99_uphill/map_origin.yaml` carries the geodetic anchor — feed
both to Autoware's `map_loader` and the simulator knows where every point
sits in WGS84. `lanelet2_map.osm` is already in geodetic lat/lon natively.

## About the TM99 sample

**TM99** stands for **T**ianmen **M**ountain **99**-Turn — the famously serpentine
hill-climb road carved into the cliffs of Tianmen Mountain in Zhangjiajie,
Hunan, China. The bag in `data/TM99_uphill/` is one full uphill run captured
with an RTK-GPS / SPAN-INS / Hesai-LiDAR / dual-camera stack:

- **31 minutes** of recording, **1.94 M** messages across **17 topics**
- **4.7 km** path, **680 m** elevation gain (327 m → 1007 m)
- 174 k INSPVAX poses (≈100 Hz, RTK-fixed most of the time)
- 18 k LiDAR scans (Hesai 64-line, 115 k points / scan, ≈3 MB each)
- 18 k H.264-compressed images per camera (front + secondary)
- IMU (raw + corrected) and EKF-fused pose at 100–1000 Hz

The TM99 dataset was **collected and provided by [Whale Dynamic](https://whaledynamic.com/)**,
a sponsor of the **Hitch Open World AI Championships**. It is included here
(under `data/TM99_uphill/`) as a runnable end-to-end sample for the pipeline.

## Credits

Pipeline assembled by **Allen Yang** (allenyang@berkeley.edu).

Sample dataset (TM99 uphill run): **Whale Dynamic**, contributed in support of
the **Hitch Open World AI Championships**.

## License

See `LICENSE` (TBD). The sample dataset under `data/TM99_uphill/` retains its
original ownership by Whale Dynamic — please refer to their terms before
redistribution.
