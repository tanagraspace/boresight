#!/usr/bin/env bash
# Everything for the pass lives in pass.toml; the command is just --config.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bin="${BORESIGHT_BIN:-$here/../../target/release/boresight}"
exec "$bin" --config "$here/pass.toml" "$@"
