"""Convert a ROS 2 .db3 bag to MCAP **with** /tf_static baked in, in one pass.

What this does in a single streaming sweep over the SQLite source:

  1. Builds a typestore including the six vendor `starneto_gps_msgs` types
     (definitions in ./msg_defs/) and emits canonical ROS 2 schema text for
     every topic. CDR bytes are otherwise passed through verbatim — values
     are bit-identical to the source.

  2. Reads the official Whale Dynamic sensor extrinsics from
     ./sensor_tf_no_camera.yaml (override with --sensor-tf, or skip
     entirely with --no-tf) and writes them as a single /tf_static message
     at bag-start. Foxglove honours /tf_static immediately, regardless of
     where in the timeline you scrub. The pp7_odom block is skipped by
     default; pass --include-pp7-odom to publish it too.

  3. Rewrites header.frame_id on /lidar_points_2_compressed messages from
     'hesai_lidar' to 'hesai_lidar_secondary' (fast byte surgery on the
     CDR string field — no full deserialise) so the two lidar streams
     resolve to distinct mounts in the TF tree rooted at `imu`.

The output is one bag — `cleaned_tf.mcap` — that's directly usable by
Foxglove and by mcap_to_SIM/build_map.py. There's no intermediate file.

Run
---
    python3 convert_db3_to_mcap.py \\
        --src ../data/TM99_uphill/raw.db3 \\
        --dst ../data/TM99_uphill/cleaned_tf.mcap
"""
from __future__ import annotations

import argparse
import os
import sqlite3
import struct
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

from mcap.writer import Writer, CompressionType
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_schema import generate_ros2_schema


CUSTOM_MSG_DIR = Path(__file__).resolve().parent / 'msg_defs'

# Reconstructed type definitions for the six vendor `starneto_gps_msgs` types.
# Order doesn't matter — rosbags resolves dependencies by name.
CUSTOM_TYPES: List[Tuple[str, Path]] = [
    ('starneto_gps_msgs/msg/NovatelMessageHeader',          CUSTOM_MSG_DIR / 'NovatelMessageHeader.msg'),
    ('starneto_gps_msgs/msg/NovatelExtendedSolutionStatus', CUSTOM_MSG_DIR / 'NovatelExtendedSolutionStatus.msg'),
    ('starneto_gps_msgs/msg/NovatelReceiverStatus',         CUSTOM_MSG_DIR / 'NovatelReceiverStatus.msg'),
    ('starneto_gps_msgs/msg/NovatelSignalMask',             CUSTOM_MSG_DIR / 'NovatelSignalMask.msg'),
    ('starneto_gps_msgs/msg/Inspvax',                       CUSTOM_MSG_DIR / 'Inspvax.msg'),
    ('starneto_gps_msgs/msg/Inspva',                        CUSTOM_MSG_DIR / 'Inspva.msg'),
    ('starneto_gps_msgs/msg/NovatelPosition',               CUSTOM_MSG_DIR / 'NovatelPosition.msg'),
    ('starneto_gps_msgs/msg/NovatelVelocity',               CUSTOM_MSG_DIR / 'NovatelVelocity.msg'),
    ('starneto_gps_msgs/msg/NovatelCorrectedImuData',       CUSTOM_MSG_DIR / 'NovatelCorrectedImuData.msg'),
    ('starneto_gps_msgs/msg/Rawimu',                        CUSTOM_MSG_DIR / 'Rawimu.msg'),
]

DEFAULT_SENSOR_TF_PATH = Path(__file__).resolve().parent / 'sensor_tf_no_camera.yaml'

DEFAULT_SECONDARY_SOURCE_TOPIC = '/lidar_points_2_compressed'
DEFAULT_SECONDARY_TARGET_FRAME = 'hesai_lidar_secondary'


# ---------- typestore + schema text -------------------------------------------

def build_typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    for tname, path in CUSTOM_TYPES:
        ts.register(get_types_from_msg(path.read_text(), tname))
    return ts


def schema_text_for(ts, typename: str) -> str:
    return generate_ros2_schema(ts, typename)


# ---------- byte surgery: rewrite frame_id in a PointCloud2 CDR ---------------

def rewrite_pc2_frame_id(cdr: bytes, new_name: str) -> bytes:
    """Replace `header.frame_id` in a sensor_msgs/PointCloud2 CDR message.

    PointCloud2 CDR layout:
        bytes  0..3  CDR encapsulation header (representation id + options)
        bytes  4..7  stamp.sec       (uint32, payload pos 0)
        bytes  8..11 stamp.nanosec   (uint32, payload pos 4)
        bytes 12..15 frame_id length (uint32, payload pos 8) — counts the null
        bytes 16..   frame_id chars + null terminator
        + padding to 4-byte alignment (relative to payload start)
        + height, width, …

    We rewrite bytes 12..(end of padding) and splice the new string in,
    recomputing alignment padding. Everything after — height, width, fields
    array, and the giant point-data byte array — is appended verbatim.
    """
    old_len = struct.unpack_from('<I', cdr, 12)[0]
    end_string_payload = 12 + old_len
    pad_old = (-end_string_payload) & 3
    rest = cdr[4 + end_string_payload + pad_old:]

    new_str = new_name.encode('utf-8') + b'\x00'
    new_len = len(new_str)
    pad_new = (-(12 + new_len)) & 3
    return (cdr[:12]
            + struct.pack('<I', new_len)
            + new_str
            + b'\x00' * pad_new
            + rest)


# ---------- TFMessage construction --------------------------------------------

def transforms_from_yaml(path: str, include_pp7_odom: bool
                         ) -> List[Tuple[str, str, list, list]]:
    """Read sensor_tf_no_camera.yaml and produce a list of (parent, child, t, q)
    transforms ready for /tf_static. Skips:
      - the `imu` block (identity-to-self has no meaning)
      - the `pp7_odom` block unless include_pp7_odom=True
    """
    doc = yaml.safe_load(Path(path).read_text())
    out: List[Tuple[str, str, list, list]] = []
    for key, block in doc.get('sensors', {}).items():
        if key == 'imu':
            continue
        if key == 'pp7_odom' and not include_pp7_odom:
            continue
        parent = block.get('parent', doc.get('reference_frame', 'imu'))
        child = block.get('child_frame_id', key)
        out.append((parent, child,
                    list(block['translation_m']),
                    list(block['rotation_quat_xyzw'])))
    return out


def build_tf_static_cdr(ts, t_ns: int,
                        transforms: List[Tuple[str, str, list, list]]) -> bytes:
    """Serialise a tf2_msgs/msg/TFMessage with the given parent→child entries."""
    TFMessage        = ts.types['tf2_msgs/msg/TFMessage']
    TransformStamped = ts.types['geometry_msgs/msg/TransformStamped']
    Transform        = ts.types['geometry_msgs/msg/Transform']
    Vector3          = ts.types['geometry_msgs/msg/Vector3']
    Quaternion       = ts.types['geometry_msgs/msg/Quaternion']
    Header           = ts.types['std_msgs/msg/Header']
    Time             = ts.types['builtin_interfaces/msg/Time']

    sec = int(t_ns // 1_000_000_000)
    nsec = int(t_ns %  1_000_000_000)

    arr = [
        TransformStamped(
            header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=parent),
            child_frame_id=child,
            transform=Transform(
                translation=Vector3(x=float(t[0]), y=float(t[1]), z=float(t[2])),
                rotation=Quaternion(x=float(q[0]), y=float(q[1]),
                                     z=float(q[2]), w=float(q[3])),
            ),
        )
        for parent, child, t, q in transforms
    ]
    return ts.serialize_cdr(TFMessage(transforms=arr), 'tf2_msgs/msg/TFMessage')


# ---------- main --------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--src', required=True, help='source .db3 file')
    p.add_argument('--dst', required=True, help='destination .mcap file '
                   '(by convention, name it cleaned_tf.mcap)')
    p.add_argument('--sensor-tf', default=str(DEFAULT_SENSOR_TF_PATH),
                   help=f'YAML with sensor extrinsics. Default: '
                        f'{DEFAULT_SENSOR_TF_PATH.name} next to this script.')
    p.add_argument('--no-tf', action='store_true',
                   help="Don't inject /tf_static and don't rewrite the secondary "
                        'lidar frame_id. Pure schema-embedding conversion only.')
    p.add_argument('--include-pp7-odom', action='store_true',
                   help='Publish the pp7_odom block in /tf_static. Off by '
                        'default per current project scope.')
    p.add_argument('--secondary-source-topic', default=DEFAULT_SECONDARY_SOURCE_TOPIC,
                   help='Topic whose header.frame_id should be rewritten.')
    p.add_argument('--secondary-target-frame-id', default=DEFAULT_SECONDARY_TARGET_FRAME,
                   help="New frame_id stamped onto secondary lidar messages. "
                        "Should match lidar_secondary's child_frame_id in the YAML.")
    p.add_argument('--max-duration-sec', type=float, default=None,
                   help='Only convert this many seconds from the start (slice mode).')
    p.add_argument('--compression', default='zstd', choices=['none', 'lz4', 'zstd'])
    p.add_argument('--chunk-size', type=int, default=4 * 1024 * 1024,
                   help='MCAP chunk size in bytes (default 4 MiB).')
    return p.parse_args()


def main():
    args = parse_args()

    print(f'[+] building typestore...', flush=True)
    ts = build_typestore()

    transforms: List[Tuple[str, str, list, list]] = []
    if not args.no_tf:
        if not Path(args.sensor_tf).exists():
            sys.exit(f'--sensor-tf path does not exist: {args.sensor_tf}\n'
                     f'    (pass --no-tf to skip /tf_static injection)')
        transforms = transforms_from_yaml(args.sensor_tf, args.include_pp7_odom)
        print(f'[+] sensor_tf:  {args.sensor_tf}', flush=True)
        for parent, child, t, q in transforms:
            print(f'    {parent} -> {child:25s}  '
                  f't=({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}) m  '
                  f'q=({q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f})',
                  flush=True)

    print(f'[+] opening source bag: {args.src}', flush=True)
    conn = sqlite3.connect(f'file:{args.src}?mode=ro', uri=True)
    cur = conn.cursor()

    cur.execute('SELECT id, name, type, serialization_format, offered_qos_profiles '
                'FROM topics ORDER BY id;')
    topics = cur.fetchall()

    cur.execute('SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages;')
    t_min, t_max, n_total = cur.fetchone()
    print(f'    bag duration: {(t_max - t_min) / 1e9:.3f}s, {n_total} messages',
          flush=True)

    t_cutoff = None
    if args.max_duration_sec:
        t_cutoff = t_min + int(args.max_duration_sec * 1e9)
        cur.execute('SELECT COUNT(*) FROM messages WHERE timestamp <= ?;', (t_cutoff,))
        n_total = cur.fetchone()[0]
        print(f'    slice: first {args.max_duration_sec}s = {n_total} messages',
              flush=True)

    print(f'[+] opening destination: {args.dst}', flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.dst)) or '.', exist_ok=True)

    compression = {
        'none': CompressionType.NONE,
        'lz4':  CompressionType.LZ4,
        'zstd': CompressionType.ZSTD,
    }[args.compression]

    with open(args.dst, 'wb') as f_out:
        writer = Writer(f_out, chunk_size=args.chunk_size, compression=compression)
        writer.start(profile='ros2', library='cowork-db3-to-mcap-tf/2.0')

        # Register schemas + channels for every original topic. Remember the
        # secondary lidar's topic_id so we can rewrite its frame_id below.
        topic_to_chan: Dict[int, int] = {}
        secondary_topic_id = None
        for tid, name, msgtype, ser_fmt, qos in topics:
            try:
                schema_text = schema_text_for(ts, msgtype)
            except Exception as e:
                print(f'    [!] cannot resolve schema for {msgtype} on {name}: {e}',
                      file=sys.stderr)
                continue
            schema_id = writer.register_schema(
                name=msgtype, encoding='ros2msg',
                data=schema_text.encode('utf-8'))
            channel_id = writer.register_channel(
                topic=name, message_encoding='cdr',
                schema_id=schema_id,
                metadata={'offered_qos_profiles': qos or ''})
            topic_to_chan[tid] = channel_id
            if name == args.secondary_source_topic and not args.no_tf:
                secondary_topic_id = tid
            print(f'    registered  {name:42s}  {msgtype}', flush=True)

        # Register /tf_static and write a single message at bag-start
        if transforms:
            tf_schema_text = generate_ros2_schema(ts, 'tf2_msgs/msg/TFMessage')
            tf_schema_id = writer.register_schema(
                name='tf2_msgs/msg/TFMessage', encoding='ros2msg',
                data=tf_schema_text.encode('utf-8'))
            tf_chan_id = writer.register_channel(
                topic='/tf_static', message_encoding='cdr',
                schema_id=tf_schema_id, metadata={})
            tf_cdr = build_tf_static_cdr(ts, t_min, transforms)
            writer.add_message(channel_id=tf_chan_id, log_time=t_min,
                                publish_time=t_min, data=tf_cdr, sequence=0)
            print(f'[+] /tf_static written ({len(transforms)} transforms, '
                  f'{len(tf_cdr)} bytes CDR)', flush=True)

        # Stream messages, rewriting secondary frame_id on the fly
        sql = 'SELECT topic_id, timestamp, data FROM messages'
        params: list = []
        if t_cutoff is not None:
            sql += ' WHERE timestamp <= ?'
            params.append(t_cutoff)
        sql += ' ORDER BY timestamp'
        cur.execute(sql, params)

        n = 0
        n_renamed = 0
        t0 = time.time()
        last_print = t0
        for topic_id, ts_ns, data in cur:
            chan = topic_to_chan.get(topic_id)
            if chan is None:
                continue
            if topic_id == secondary_topic_id:
                data = rewrite_pc2_frame_id(data, args.secondary_target_frame_id)
                n_renamed += 1
            writer.add_message(channel_id=chan, log_time=ts_ns,
                                publish_time=ts_ns, data=data, sequence=n)
            n += 1
            now = time.time()
            if now - last_print > 5.0:
                rate = n / (now - t0)
                pct = 100.0 * n / max(n_total, 1)
                eta = (n_total - n) / rate if rate > 0 else 0
                print(f'    [{pct:5.1f}%] {n:,}/{n_total:,} msgs   '
                      f'{rate:.0f} msg/s   ETA {eta:.0f}s', flush=True)
                last_print = now

        writer.finish()
        print(f'[+] wrote {n:,} messages in {time.time()-t0:.1f}s '
              f'({n_renamed:,} secondary-lidar frame_id rewrites)', flush=True)

    conn.close()
    sz = os.path.getsize(args.dst)
    print(f'[+] dst size: {sz/1e9:.2f} GB ({sz} bytes)', flush=True)


if __name__ == '__main__':
    main()
