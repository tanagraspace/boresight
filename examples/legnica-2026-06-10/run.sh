#!/usr/bin/env bash
# Example pass: OPS-SAT PRETTY (NORAD 58023) over Legnica, Poland, 2026-06-10.
# A high-elevation pass (~73 deg peak). Run from the repo root after a build:
#   cargo build --release && examples/legnica-2026-06-10/run.sh
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bin="${BORESIGHT_BIN:-$here/../../target/release/boresight}"

exec "$bin" \
  --attitude "$here/attitude.csv" \
  --time-col "time" --quat-cols "ukf_X x,ukf_X y,ukf_X z,ukf_X k" \
  --tle1 "1 58023U 23155H   26161.48719104  .00004557  00000+0  19207-3 0  9996" \
  --tle2 "2 58023  97.5689 243.0215 0003014  75.7594 284.3977 15.23841600147334" \
  --target-ecef "3845782.0,1114412.25,4948097.5" --target-name Legnica \
  --reference 2026-06-10T21:21:20Z \
  --marker "experiment start=2026-06-10T21:21:23Z" \
  --carrier-hz 1296000000 \
  --windows "9.86:29.86,33.04:53.04,55.78:75.78,79.44:99.44,102.43:122.43,126.02:146.02" \
  "$@"
