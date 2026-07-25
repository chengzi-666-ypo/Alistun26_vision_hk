#!/usr/bin/env bash
# one-click runner for the full auto-aim system using SERIAL communication
# Usage: CONFIG=configs/standard_serial.yaml BINARY=build/mt_standard CBOARD_PORT=/dev/ttyACM0 GIMBAL_PORT=/dev/ttyACM1 ./scripts/run_serial.sh

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="$ROOT_DIR/logs/run"
mkdir -p "$LOGDIR"

# Default configuration
CBOARD_PORT=${CBOARD_PORT:-/dev/ttyACM0}
GIMBAL_PORT=${GIMBAL_PORT:-/dev/ttyACM1}
CONFIG=${CONFIG:-configs/standard_serial.yaml}
BINARY=${BINARY:-build/mt_standard}

echo "[run_serial] ROOT_DIR=$ROOT_DIR"
echo "[run_serial] CBOARD_PORT=$CBOARD_PORT, GIMBAL_PORT=$GIMBAL_PORT, CONFIG=$CONFIG, BINARY=$BINARY"

# Ensure CBOARD_PORT exists and link it to /dev/cboard
if [[ -e "$CBOARD_PORT" ]]; then
  echo "[run_serial] linking /dev/cboard -> $CBOARD_PORT"
  sudo ln -sf "$CBOARD_PORT" /dev/cboard
  # Optional: Set permissions
  sudo chmod 666 "$CBOARD_PORT"
else
  echo "[run_serial] WARNING: CBOARD_PORT $CBOARD_PORT does not exist"
fi

# Ensure GIMBAL_PORT exists and link it to /dev/gimbal
if [[ -e "$GIMBAL_PORT" ]]; then
  echo "[run_serial] linking /dev/gimbal -> $GIMBAL_PORT"
  sudo ln -sf "$GIMBAL_PORT" /dev/gimbal
  # Optional: Set permissions
  sudo chmod 666 "$GIMBAL_PORT"
else
  echo "[run_serial] WARNING: GIMBAL_PORT $GIMBAL_PORT does not exist"
fi

# Start the main binary
echo "[run_serial] starting main: $BINARY $CONFIG"
mkdir -p "$ROOT_DIR/logs/run"
# 切换到项目根目录运行，确保能找到 assets/ 下的模型文件
cd "$ROOT_DIR"
nohup "$ROOT_DIR/$BINARY" "$ROOT_DIR/$CONFIG" > "$LOGDIR/main.log" 2>&1 &
echo "[run_serial] main PID $! (logging to $LOGDIR/main.log)"

echo "[run_serial] All started. Check logs in $LOGDIR"
echo "To stop: pkill -f $(basename "$BINARY") || true"
