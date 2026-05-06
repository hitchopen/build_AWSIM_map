# db3 → MCAP converter for `TM99_uphill/raw.db3`

Converts the ROS2 SQLite-3 bag at `data/TM99_uphill/raw.db3` (96 GB, 1.94 M
messages, 17 topics) to MCAP. CDR bytes are passed through verbatim — values
are bit-identical to the source — and schemas are embedded in the canonical
ROS2 form so Foxglove Studio decodes every topic natively, including the six
vendor `starneto_gps_msgs` topics that the .db3 layout couldn't carry.

## What's in the source bag

- 1,940,121 messages, ≈31 minutes (1860 s)
- 17 topics, GPS in Hunan, China (29.07° N, 110.47° E)
- 11 topics use well-known ROS2 messages (`sensor_msgs`, `geometry_msgs`,
  `nav_msgs`)
- 6 topics use vendor messages from `starneto_gps_msgs`. The .db3 layout
  doesn't carry message definitions, which is why Foxglove can't decode them
  directly from the bag.

## How `starneto_gps_msgs` is handled

`starneto_gps_msgs` is a clone of SwRI's `novatel_gps_msgs` package — every
field name and order matches, and the byte layout we observed in the bag
decoded cleanly against SwRI's definitions. The script ships five SwRI
definitions (`Inspvax`, `Inspva`, `NovatelPosition`, `NovatelVelocity`,
`NovatelCorrectedImuData`) plus their dependencies (`NovatelMessageHeader`,
`NovatelReceiverStatus`, `NovatelExtendedSolutionStatus`, `NovatelSignalMask`).

The sixth type, `Rawimu`, isn't published by SwRI's package; its byte layout
was reverse-engineered from sample messages and confirmed against expected
physical values (z accel ≈ 9.8 m/s², gyro rates ≈ 1e-3 rad/s for a stationary
vehicle). The reconstructed definition is in `msg_defs/Rawimu.msg`.

The `gps_L1`-style fields in `NovatelSignalMask` were renamed to lowercase
to satisfy ROS2's field-name regex (uppercase letters aren't legal). This
is purely cosmetic — CDR encoding is name-agnostic.

## Install dependencies

```bash
pip install --user mcap mcap-ros2-support rosbags
```

Python ≥ 3.10. No ROS install needed — `rosbags` is a pure-Python typestore.

## Run the full conversion

```bash
cd /Users/yang/Documents/GitHub/build_AWSIM_map/db3_to_mcap_converter

python3 convert_db3_to_mcap.py \
    --src ../data/TM99_uphill/raw.db3 \
    --dst ../data/TM99_uphill/cleaned.mcap \
    --compression none
```

Expected runtime: ≈60–70 min on a modern laptop. Expected output size: ≈96 GB
(uncompressed; pass `--compression zstd` to drop to ~50–60 GB at the cost of
some CPU time and slightly slower scrubbing in Foxglove).

## Filter or slice

```bash
# First 30 seconds only
python3 convert_db3_to_mcap.py \
    --src ../data/TM99_uphill/raw.db3 \
    --dst /tmp/first_30s.mcap \
    --max-duration-sec 30 \
    --compression zstd
```

## Verify the output

```bash
python3 - <<'PY'
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
with open('../data/TM99_uphill/cleaned.mcap','rb') as f:
    r = make_reader(f, decoder_factories=[DecoderFactory()])
    s = r.get_summary()
    print(f"channels={len(s.channels)}  msgs={s.statistics.message_count}")
    for cid, n in sorted(s.statistics.channel_message_counts.items()):
        c = s.channels[cid]
        print(f"  {n:7d}  {c.topic}  ->  {s.schemas[c.schema_id].name}")
PY
```

## Files

```
convert_db3_to_mcap.py   The converter (.db3 → .mcap, CLI entry point)
ros2_schema.py           Builds canonical ROS2 msg-def text from a rosbags
                         Typestore (builtin_interfaces/Time, unqualified
                         same-package refs, spaces around `=`).
msg_defs/                Reconstructed .msg files for starneto_gps_msgs.
```
