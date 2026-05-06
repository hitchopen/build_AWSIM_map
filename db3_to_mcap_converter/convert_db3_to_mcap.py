"""
Convert ROS2 .db3 bag to MCAP, embedding schemas for both well-known
and starneto_gps_msgs types so Foxglove can natively decode every topic.

CDR bytes are passed through verbatim — no re-serialization, so values are
guaranteed identical to the source.
"""

import argparse
import os
import sqlite3
import sys
import time
from pathlib import Path

from mcap.writer import Writer, CompressionType
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_schema import generate_ros2_schema


CUSTOM_MSG_DIR = Path(__file__).resolve().parent / 'msg_defs'

# Map source typenames (as they appear in the .db3) → our reconstructed .msg files.
# Each source type points to (typename used for registration, .msg file).
CUSTOM_TYPES = [
    ('starneto_gps_msgs/msg/NovatelMessageHeader',        CUSTOM_MSG_DIR / 'NovatelMessageHeader.msg'),
    ('starneto_gps_msgs/msg/NovatelExtendedSolutionStatus', CUSTOM_MSG_DIR / 'NovatelExtendedSolutionStatus.msg'),
    ('starneto_gps_msgs/msg/NovatelReceiverStatus',       CUSTOM_MSG_DIR / 'NovatelReceiverStatus.msg'),
    ('starneto_gps_msgs/msg/NovatelSignalMask',           CUSTOM_MSG_DIR / 'NovatelSignalMask.msg'),
    ('starneto_gps_msgs/msg/Inspvax',                     CUSTOM_MSG_DIR / 'Inspvax.msg'),
    ('starneto_gps_msgs/msg/Inspva',                      CUSTOM_MSG_DIR / 'Inspva.msg'),
    ('starneto_gps_msgs/msg/NovatelPosition',             CUSTOM_MSG_DIR / 'NovatelPosition.msg'),
    ('starneto_gps_msgs/msg/NovatelVelocity',             CUSTOM_MSG_DIR / 'NovatelVelocity.msg'),
    ('starneto_gps_msgs/msg/NovatelCorrectedImuData',     CUSTOM_MSG_DIR / 'NovatelCorrectedImuData.msg'),
    ('starneto_gps_msgs/msg/Rawimu',                      CUSTOM_MSG_DIR / 'Rawimu.msg'),
]


def build_typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    for tname, path in CUSTOM_TYPES:
        ts.register(get_types_from_msg(path.read_text(), tname))
    return ts


def to_rosbags_name(t):
    """'sensor_msgs/msg/Imu'  →  'sensor_msgs/msg/Imu' (rosbags uses /msg/ form already)."""
    return t


def schema_text_for(ts, typename):
    """Generate the concatenated ROS2 msg-def text for `typename` (e.g. 'sensor_msgs/msg/Imu')."""
    return generate_ros2_schema(ts, to_rosbags_name(typename))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--src', required=True, help='source .db3 file')
    p.add_argument('--dst', required=True, help='destination .mcap file')
    p.add_argument('--max-duration-sec', type=float, default=None,
                   help='only convert this many seconds from the start (for testing)')
    p.add_argument('--compression', default='zstd', choices=['none','lz4','zstd'])
    p.add_argument('--chunk-size', type=int, default=4 * 1024 * 1024,
                   help='MCAP chunk size in bytes (default 4 MiB)')
    args = p.parse_args()

    print(f'[+] building typestore...', flush=True)
    ts = build_typestore()

    print(f'[+] opening source bag: {args.src}', flush=True)
    conn = sqlite3.connect(f'file:{args.src}?mode=ro', uri=True)
    cur = conn.cursor()

    cur.execute('SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics ORDER BY id;')
    topics = cur.fetchall()

    cur.execute('SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages;')
    t_min, t_max, n_total = cur.fetchone()
    print(f'    bag duration: {(t_max-t_min)/1e9:.3f}s, {n_total} messages', flush=True)

    t_cutoff = None
    if args.max_duration_sec:
        t_cutoff = t_min + int(args.max_duration_sec * 1e9)
        cur.execute('SELECT COUNT(*) FROM messages WHERE timestamp <= ?;', (t_cutoff,))
        n_total = cur.fetchone()[0]
        print(f'    slice: first {args.max_duration_sec}s = {n_total} messages', flush=True)

    print(f'[+] opening destination: {args.dst}', flush=True)
    os.makedirs(os.path.dirname(args.dst), exist_ok=True)

    compression = {
        'none': CompressionType.NONE,
        'lz4':  CompressionType.LZ4,
        'zstd': CompressionType.ZSTD,
    }[args.compression]

    with open(args.dst, 'wb') as f_out:
        writer = Writer(f_out, chunk_size=args.chunk_size, compression=compression)
        writer.start(profile='ros2', library='cowork-db3-to-mcap/1.0')

        # Register schemas + channels
        topic_to_chan = {}
        for tid, name, msgtype, ser_fmt, qos in topics:
            try:
                schema_text = schema_text_for(ts, msgtype)
            except Exception as e:
                print(f'    [!] cannot resolve schema for {msgtype} on {name}: {e}', file=sys.stderr)
                continue

            schema_id = writer.register_schema(
                name=msgtype,
                encoding='ros2msg',
                data=schema_text.encode('utf-8'),
            )
            channel_id = writer.register_channel(
                topic=name,
                message_encoding='cdr',
                schema_id=schema_id,
                metadata={'offered_qos_profiles': qos or ''},
            )
            topic_to_chan[tid] = channel_id
            print(f'    registered  {name:42s}  {msgtype}', flush=True)

        # Stream messages
        sql = 'SELECT topic_id, timestamp, data FROM messages'
        params = []
        if t_cutoff is not None:
            sql += ' WHERE timestamp <= ?'
            params.append(t_cutoff)
        sql += ' ORDER BY timestamp'

        cur.execute(sql, params)

        n = 0
        t0 = time.time()
        last_print = t0
        for topic_id, ts_ns, data in cur:
            chan = topic_to_chan.get(topic_id)
            if chan is None:
                continue
            writer.add_message(
                channel_id=chan,
                log_time=ts_ns,
                publish_time=ts_ns,
                data=data,
                sequence=n,
            )
            n += 1
            now = time.time()
            if now - last_print > 5.0:
                rate = n / (now - t0)
                pct = 100.0 * n / max(n_total, 1)
                eta = (n_total - n) / rate if rate > 0 else 0
                print(f'    [{pct:5.1f}%] {n}/{n_total} msgs   {rate:.0f} msg/s   ETA {eta:.0f}s', flush=True)
                last_print = now

        writer.finish()
        print(f'[+] wrote {n} messages in {time.time()-t0:.1f}s', flush=True)

    conn.close()
    sz = os.path.getsize(args.dst)
    print(f'[+] dst size: {sz/1e9:.2f} GB ({sz} bytes)', flush=True)


if __name__ == '__main__':
    main()
