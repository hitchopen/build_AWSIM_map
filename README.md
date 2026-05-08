# build_AWSIM_map

Tooling that turns a real-world ROS 2 sensor recording into the three artefacts
AWSIM (and Autoware behind it) needs to load a track for self-driving simulation:

- **`pointcloud_map.pcd`** — LiDAR-built point cloud of the environment, used
  by Autoware's NDT matching for localisation.
- **`lanelet2_map.osm`** — vector HD map (centreline + lane boundaries) used
  by the planner and the simulator for routing and behaviour.
- **`map_origin.yaml`** — the geodetic anchor (lat / lon / alt) for the local
  ENU frame the rest of the pipeline lives in.

The pipeline runs in two stages — both written for a stock Python install,
no ROS or PCL required:

```
data/<track>/raw.db3                            (input: ROS 2 SQLite-3 bag)
                │
                │  db3_to_mcap_converter/convert_db3_to_mcap.py
                ▼
data/<track>/cleaned.mcap                       (Foxglove-loadable MCAP)
                │
                │  mcap_to_SIM/build_map.py
                ▼
map/<track>/{pointcloud_map.pcd,
             lanelet2_map.osm,
             map_origin.yaml,
             trajectory_enu.csv}                (AWSIM-ready map)
```

## Repository layout

```
build_AWSIM_map/
├── data/                       Sample sensor recordings (fetched on demand)
│   └── TM99_uphill/
│       ├── download.sh             pulls the two large blobs from OneDrive
│       ├── raw.db3                 ROS 2 humble bag, SQLite-3 storage  (~89 GiB, gitignored)
│       └── cleaned.mcap            same data, repacked as MCAP         (~89 GiB, gitignored)
├── db3_to_mcap_converter/      .db3 → .mcap (CDR bytes pass through verbatim,
│                               schemas re-emitted in canonical ROS 2 form so
│                               Foxglove decodes every topic — including six
│                               vendor types from `starneto_gps_msgs` that the
│                               .db3 layout couldn't carry)
├── mcap_to_SIM/                .mcap → AWSIM map (pose extraction, voxel-grid
│                               point cloud accumulation, Lanelet2 OSM seed)
└── map/                        Build outputs (gitignored — regenerable)
    └── TM99_uphill/
```

Each subdirectory has its own README with the run commands, expected runtime,
and tunables.

## Quick start

The TM99 sample bag (~180 GiB total) isn't stored in the repo — fetch it
from the sponsor's OneDrive mirror once before running anything else:

```bash
bash data/TM99_uphill/download.sh
```

```bash
# 1. one-off: convert the source bag
cd db3_to_mcap_converter
pip install --user mcap mcap-ros2-support rosbags
python3 convert_db3_to_mcap.py \
    --src ../data/TM99_uphill/raw.db3 \
    --dst ../data/TM99_uphill/cleaned.mcap \
    --compression none      # ~70 min, ~96 GB output

# 2. build the AWSIM map
cd ../mcap_to_SIM
pip install --user -r requirements.txt
python3 build_map.py \
    --mcap ../data/TM99_uphill/cleaned.mcap \
    --out-dir ../map/TM99_uphill \
    --voxel-size 0.2 \
    --lidar-decimate 5      # ~5–10 min
```

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
