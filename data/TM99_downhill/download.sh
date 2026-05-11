#!/usr/bin/env bash
# Download the TM99_downhill source recording archive.
#
# raw.tar.xz contains the downhill source data for the TM99 sample track. It is
# too large for GitHub/LFS, so it is hosted on OneDrive via an "Anyone with the
# link" share URL. Run this script to fetch it into this directory.
#
# Usage:  bash download.sh
#
# Re-running is safe. An already-complete raw.tar.xz matching the recorded byte
# count is skipped, and partial downloads are resumed in place.

set -euo pipefail

cd "$(dirname "$0")"

# OneDrive sharing URL. Appending `&download=1` swaps the in-browser viewer for
# a direct file blob, so curl gets the bytes instead of the HTML page.
URL_RAW_ARCHIVE="https://intelligentracingcom-my.sharepoint.com/:u:/g/personal/allen_intelligentracing_com/IQCOlENiwJCPQZgTJNjKObMCAUcIsFkjb_yqzQC-fRX1UW4?e=otWuG3&download=1"

# Expected size in bytes, used to detect truncated downloads.
SIZE_RAW_ARCHIVE=29726208932      # 27.68 GiB

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
        echo "[part] $out  $(human "$have") of $(human "$expected") - resuming"
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
        echo "       Try re-running this script - curl will resume." >&2
        exit 1
    fi
    echo "[ok  ] $out  $(human "$actual")"
}

download "$URL_RAW_ARCHIVE" raw.tar.xz "$SIZE_RAW_ARCHIVE"

echo
echo "Source archive downloaded to $(pwd)/raw.tar.xz"
echo
echo "Extract it in this folder when you need the raw data:"
echo "  tar -xf raw.tar.xz"
