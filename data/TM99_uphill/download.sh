#!/usr/bin/env bash
# Download the TM99_uphill sample sensor recording.
#
# These two files (~90 GB each) are too large for GitHub LFS, so they are
# hosted on OneDrive via "Anyone with the link" share URLs. Run this script
# to fetch them into the same directory; the rest of the pipeline (the
# db3_to_mcap_converter and mcap_to_SIM tools) reads from these paths
# directly.
#
# Usage:  bash download.sh
#
# Re-running is safe — already-complete files (matching the recorded byte
# count) are skipped, and partial downloads are resumed in place.

set -euo pipefail

cd "$(dirname "$0")"

# OneDrive sharing URLs. Appending `&download=1` swaps the in-browser viewer
# for a direct file blob, so curl gets the bytes instead of the HTML page.
URL_RAW_DB3="https://intelligentracingcom-my.sharepoint.com/:u:/g/personal/allen_intelligentracing_com/IQATmpMEtsWRQ7TRu5G5bd4rAc-4bW6dETdFuV_JeKSOobA?e=MLwoFS&download=1"
URL_CLEANED_MCAP="https://intelligentracingcom-my.sharepoint.com/:u:/g/personal/allen_intelligentracing_com/IQCWm67x8_3kQLL-M6LmUQNLAfsSjxJZ6Hg4k4dbaN7uhHk?e=p7nhGG&download=1"

# Expected sizes in bytes — used to detect truncated downloads.
SIZE_RAW_DB3=96008151040          # 89.4 GiB
SIZE_CLEANED_MCAP=95854412308     # 89.3 GiB

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

download "$URL_RAW_DB3"      raw.db3      "$SIZE_RAW_DB3"
download "$URL_CLEANED_MCAP" cleaned.mcap "$SIZE_CLEANED_MCAP"

echo
echo "All sample data is in $(pwd)"
echo
echo "Optional integrity check (slow — reads ~180 GiB):"
echo "  shasum -a 256 raw.db3 cleaned.mcap > checksums.txt"
