#!/usr/bin/env bash
# one-click runner for the simple serial aim system
# Usage: CBOARD_PORT=/dev/ttyACM0 ./scripts/run_simple.sh

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="$ROOT_DIR/logs/run"
mkdir -p "$LOGDIR"

# Default configuration
CBOARD_PORT=${CBOARD_PORT:-/dev/ttyACM0}
CONFIG=${CONFIG:-configs/standard_serial.yaml}
BINARY=${BINARY:-build/simple_serial_aim}

echo "[run_simple] ROOT_DIR=$ROOT_DIR"
echo "[run_simple] CBOARD_PORT=$CBOARD_PORT, CONFIG=$CONFIG, BINARY=$BINARY"

# Ensure CBOARD_PORT exists and link it to /dev/cboard
if [[ -e "$CBOARD_PORT" ]]; then
  echo "[run_simple] linking /dev/cboard -> $CBOARD_PORT"
  sudo ln -sf "$CBOARD_PORT" /dev/cboard
  # Optional: Set permissions
  sudo chmod 666 "$CBOARD_PORT"
else
  echo "[run_simple] WARNING: CBOARD_PORT $CBOARD_PORT does not exist"
fi

# Start the main binary
echo "[run_simple] starting main: $BINARY --config-path=$CONFIG"

# We run this in foreground so user can see the window and output
cd "$ROOT_DIR"
./$BINARY --config-path=$CONFIG
