#!/usr/bin/env bash
set -euo pipefail

CONFIG="${1:-config/erasor2/seq_00.yaml}"
if [[ ! -f "$CONFIG" ]]; then
  echo "Config not found: $CONFIG" >&2
  exit 2
fi

exec /usr/bin/time -v ./build/run_erasor2 "$CONFIG"
