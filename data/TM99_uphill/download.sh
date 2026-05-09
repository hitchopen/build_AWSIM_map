#!/usr/bin/env bash
# Download the TM99_uphill source recording (raw.db3 only).
#
# raw.db3 is the ROS 2 SQLite-3 bag from the original recording (~89 GiB).
# It's too large for GitHub LFS, so it's hosted on OneDrive via an "Anyone
# with the link" share URL. Run this script to fetch it into the same
# directory.
#
# The MCAP variant (`cleaned_tf.mcap`) is *not* downloaded — it's a derived
# artefact regenerable from raw.db3 via:
#
#   cd ../../db3_to_mcap_converter
#   python3 convert_db3_to_mcap.py \
#       --src ../data/TM99_uphill/raw.db3 \
#       --dst ../data/TM99_uphill/cleaned_tf.mcap
#
# Usage:  bash download.sh
#
# Re-running is safe — an already-complete raw.db3 (matching the recorded
# byte count) is skipped, and partial downloads are resumed in place.

set -euo pipefail

cd "$(dirname "$0")"

# OneDrive sharing URL. Appending `&download=1` swaps the in-browser viewer
# for a direct file blob, so curl gets the bytes instead of the HTML page.
URL_RAW_DB3="https://intelligentracingcom-my.sharepoint.com/:u:/g/personal/allen_intelligentracing_com/IQATmpMEtsWRQ7TRu5G5bd4rAc-4bW6dETdFuV_JeKSOobA?e=MLwoFS&download=1"

# Expected size in bytes — used to detect truncated downloads.
SIZE_RAW_DB3=96008151040          # 89.4 GiB

# Portable file-size lookup (BSD stat on macOS, GNU stat on Linux).
filesize() {
    if stat -c%s "$1" >/dev/null 2>&1; then
        stat -c%s "$1"
    else
        stat -f%z "$1"
    fi
}

human() {
    awk -v b="$1" 'BEGIN { split("B KiB MiB GiB TiB", u); i = 1
        while (b > 1024 && i < 5) { b /= 1024; i++ }
        printf "%.2f %s", b, u[i] }'
}

download() {
    local url="$1" out="$2" expected="$3"

    if [[ -f "$out" ]]; then
        local have
        have="$(filesize "$out")"
        if [[ "$have" == "$expected" ]]; then
            echo "[skip] $out  already complete ($(human "$have"))"
            return 0
        fi
        echo "[part] $out  $(human "$have") of $(human "$expected") — resuming"
    else
        echo "[get ] $out  ($(human "$expected"))"
    fi

    # -L follows redirects (OneDrive bounces through a CDN host),
    # -C - resumes from the existing byte offset on retry,
    # --retry/--retry-all-errors handle transient network glitches.
    curl -L -C - --retry 5 --retry-all-errors --retry-delay 5 \
         -o "$out" "$url"

    local actual
    actual="$(filesize "$out")"
    if [[ "$actual" != "$expected" ]]; then
        echo "[err ] $out  size mismatch: got $actual, expected $expected" >&2
        echo "       Try re-running this script — curl will resume." >&2
        exit 1
    fi
    echo "[ok  ] $out  $(human "$actual")"
}

download "$URL_RAW_DB3" raw.db3 "$SIZE_RAW_DB3"

echo
echo "Source bag downloaded to $(pwd)/raw.db3"
echo
echo "Next step — generate the Foxglove-ready MCAP (~60–90 min):"
echo "  cd ../../db3_to_mcap_converter && python3 convert_db3_to_mcap.py \\"
echo "      --src ../data/TM99_uphill/raw.db3 \\"
echo "      --dst ../data/TM99_uphill/cleaned_tf.mcap"
echo
echo "Optional integrity check on the source bag (slow — reads ~89 GiB):"
echo "  shasum -a 256 raw.db3 > checksums.txt"
