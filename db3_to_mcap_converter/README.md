# db3 → MCAP converter (with /tf_static baked in) for `TM99_uphill`

A single script that turns the raw ROS 2 SQLite-3 bag into a Foxglove-loadable
MCAP with the official Whale Dynamic sensor extrinsics already published as
`/tf_static`. One pass over the source, one output file.

```
data/<track>/raw.db3
    │
    │  convert_db3_to_mcap.py        ── reads sensor_tf_no_camera.yaml
    ▼                                   by default; CDR bytes pass through
data/<track>/cleaned_tf.mcap            verbatim; secondary lidar's
                                        frame_id rewritten on the fly so
                                        each cloud resolves to its own
                                        mount under the `imu` TF root.
```

## What's in the source bag

- 1,940,121 messages, ≈31 minutes (1860 s)
- 17 topics, GPS in Hunan, China (29.07° N, 110.47° E)
- 11 topics use well-known ROS 2 messages (`sensor_msgs`, `geometry_msgs`,
  `nav_msgs`)
- 6 topics use vendor messages from `starneto_gps_msgs`. The .db3 layout
  doesn't carry message definitions, which is why Foxglove can't decode
  them directly from a `.db3` file.

## How `starneto_gps_msgs` is handled

`starneto_gps_msgs` is a clone of SwRI's `novatel_gps_msgs` package — every
field name and order matches, and the byte layout in the bag decoded
cleanly against SwRI's definitions. The script ships five SwRI definitions
(`Inspvax`, `Inspva`, `NovatelPosition`, `NovatelVelocity`,
`NovatelCorrectedImuData`) plus their dependencies (`NovatelMessageHeader`,
`NovatelReceiverStatus`, `NovatelExtendedSolutionStatus`,
`NovatelSignalMask`).

The sixth type, `Rawimu`, isn't published by SwRI's package; its byte
layout was reverse-engineered from sample messages and confirmed against
expected physical values (z accel ≈ 9.8 m/s², gyro rates ≈ 1e-3 rad/s for a
stationary vehicle). The reconstructed definition is in
`msg_defs/Rawimu.msg`.

The `gps_L1`-style fields in `NovatelSignalMask` were renamed to lowercase
to satisfy ROS 2's field-name regex (uppercase letters aren't legal in
ROS 2). Cosmetic — CDR encoding is name-agnostic.

## Install dependencies

```bash
pip install --user -r requirements.txt
```

Python ≥ 3.10. No ROS or PCL needed — `rosbags` is a pure-Python typestore.

## Run

```bash
cd /Users/yang/Library/CloudStorage/OneDrive-IntelligentRacingInc/GitHub/build_AWSIM_map/db3_to_mcap_converter

python3 convert_db3_to_mcap.py \
    --src ../data/TM99_uphill/raw.db3 \
    --dst ../data/TM99_uphill/cleaned_tf.mcap \
    --compression none
```

Expected runtime: ≈60–90 min on a modern laptop. Expected output size: ≈96 GB
uncompressed (pass `--compression zstd` to drop to ~50–60 GB at the cost of
some CPU time and slightly slower scrubbing in Foxglove).

In a single pass the script:

1. Embeds canonical ROS 2 schemas for every topic (including the six vendor
   `starneto_gps_msgs` types).
2. Loads `sensor_tf_no_camera.yaml` (default — `--sensor-tf` to override,
   `--no-tf` to skip) and writes a single `tf2_msgs/TFMessage` to
   `/tf_static` at bag-start. Foxglove honours `/tf_static` immediately,
   regardless of where in the timeline you scrub.
3. Rewrites `header.frame_id` on every `/lidar_points_2_compressed` message
   from `hesai_lidar` to `hesai_lidar_secondary` (fast byte surgery on the
   CDR string field) so the two lidar streams resolve to distinct mounts
   in the TF tree rooted at `imu`. The `pp7_odom` block is skipped by
   default; pass `--include-pp7-odom` to publish it too.

CDR message bytes are otherwise passed through verbatim — values are
bit-identical to the source bag.

## Useful flags

| flag | purpose |
| --- | --- |
| `--src / --dst`              | input .db3 / output .mcap (required) |
| `--sensor-tf <path>`         | override the YAML used for /tf_static. Default: `sensor_tf_no_camera.yaml` next to the script. |
| `--no-tf`                    | skip /tf_static injection AND skip the secondary-lidar frame_id rewrite — pure schema-embedding conversion only. |
| `--include-pp7-odom`         | publish the `pp7_odom` block too. Off by default. |
| `--secondary-source-topic`   | topic whose `header.frame_id` should be rewritten. Default `/lidar_points_2_compressed`. |
| `--secondary-target-frame-id`| new frame_id for those messages. Default `hesai_lidar_secondary` — should match the YAML's `lidar_secondary.child_frame_id`. |
| `--max-duration-sec`         | only convert this many seconds from bag start (slice mode for testing). |
| `--compression`              | `none` / `lz4` / `zstd`. Default `zstd`. |
| `--chunk-size`               | MCAP chunk size in bytes. Default 4 MiB. |

## Verify the output

```bash
python3 - <<'PY'
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
with open('../data/TM99_uphill/cleaned_tf.mcap','rb') as f:
    r = make_reader(f, decoder_factories=[DecoderFactory()])
    s = r.get_summary()
    print(f"channels={len(s.channels)}  msgs={s.statistics.message_count}")
    for cid, n in sorted(s.statistics.channel_message_counts.items()):
        c = s.channels[cid]
        print(f"  {n:7d}  {c.topic}  ->  {s.schemas[c.schema_id].name}")
PY
```

You should see `/tf_static` listed alongside the original 17 topics, and a
spot-check of `/lidar_points_2_compressed` messages should show
`header.frame_id = 'hesai_lidar_secondary'`.

## sensor_tf_no_camera.yaml

The official sensor extrinsics file from Whale Dynamic. All transforms are
written as **T_sensor → imu**:

```
p_imu = R · p_sensor + t
```

Convention is FLU throughout (X forward, Y left, Z up), per ROS REP-103.

| sensor | child_frame_id | mount summary |
|---|---|---|
| `lidar_primary`   | `hesai_lidar`            | rear-facing (R_z(180°)), 0.175 m right of IMU, 0.13 m up |
| `lidar_secondary` | `hesai_lidar_secondary`  | tilted ~30° from primary, 0.47 m left, 0.27 m below IMU |
| `pp7_odom`        | `pp7_odom`               | virtual frame, +90° about Z (skipped by default) |

Note: SPAN's INSPVAX (`roll, pitch, azimuth`) angles are still in NED/FRD
convention even though `imu` is FLU; downstream consumers must convert.
`mcap_to_SIM/lib_pose.py` does this conversion correctly.

## Files

```
convert_db3_to_mcap.py    .db3 → .mcap with /tf_static baked in (CLI entry point)
ros2_schema.py            builds canonical ROS 2 msg-def text from a rosbags
                           Typestore (builtin_interfaces/Time, unqualified
                           same-package refs, spaces around `=`).
sensor_tf_no_camera.yaml  official extrinsics from Whale Dynamic
msg_defs/                 reconstructed .msg files for starneto_gps_msgs
requirements.txt
```
