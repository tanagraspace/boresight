#!/usr/bin/env bash
# Minimal invocation: the only required inputs are an attitude CSV, a TLE, and a
# target. Everything else uses defaults:
#   - default column names (time, qx, qy, qz, qw),
#   - reference time defaults to the first attitude sample,
#   - no windows, no markers, no carrier (shows range rate, not Doppler),
#   - boresight +X.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bin="${BORESIGHT_BIN:-$here/../../target/release/boresight}"

exec "$bin" \
  --attitude "$here/attitude.csv" \
  --tle-file "$here/sat.tle" \
  --target-lat 51.208333 --target-lon 16.160278 \
  "$@"
