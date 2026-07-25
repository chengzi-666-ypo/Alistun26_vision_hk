#!/usr/bin/env bash
# one-click runner for the full auto-aim system (uses vcan0 by default)
# Usage: CONFIG=configs/standard3.yaml BINARY=build/mt_standard CAN_IFACE=vcan0 GIMBAL_PORT=/dev/ttyACM1 ./scripts/run_all.sh

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="$ROOT_DIR/logs/run"
mkdir -p "$LOGDIR"

CAN_IFACE=${CAN_IFACE:-vcan0}
GIMBAL_PORT=${GIMBAL_PORT:-/dev/ttyACM1}
CONFIG=${CONFIG:-configs/standard3.yaml}
BINARY=${BINARY:-build/mt_standard}

echo "[run_all] ROOT_DIR=$ROOT_DIR"
echo "[run_all] CAN_IFACE=$CAN_IFACE, GIMBAL_PORT=$GIMBAL_PORT, CONFIG=$CONFIG, BINARY=$BINARY"

# create vcan0 if requested
if [[ "$CAN_IFACE" == vcan* ]]; then
  if ! ip link show "$CAN_IFACE" >/dev/null 2>&1; then
    echo "[run_all] creating $CAN_IFACE"
    sudo modprobe vcan || true
    sudo ip link add dev "$CAN_IFACE" type vcan || true
    sudo ip link set up "$CAN_IFACE"
  else
    echo "[run_all] $CAN_IFACE already exists"
  fi
fi

# ensure /dev/gimbal symlink points to user-specified port
if [[ -e "$GIMBAL_PORT" ]]; then
  sudo ln -sf "$GIMBAL_PORT" /dev/gimbal
  echo "[run_all] linked /dev/gimbal -> $GIMBAL_PORT"
else
  echo "[run_all] WARNING: GIMBAL_PORT $GIMBAL_PORT does not exist"
fi

# start candump
echo "[run_all] starting candump on $CAN_IFACE"
nohup candump "$CAN_IFACE" > "$LOGDIR/candump.log" 2>&1 &
echo "[run_all] candump PID $! (logging to $LOGDIR/candump.log)"

# start referee listener (writes /tmp/robot_enemy_color)
if [[ -x "$ROOT_DIR/tools/referee_listener" ]]; then
  nohup "$ROOT_DIR/tools/referee_listener" "$CAN_IFACE" > "$LOGDIR/referee_listener.log" 2>&1 &
  echo "[run_all] referee_listener PID $! (logging to $LOGDIR/referee_listener.log)"
else
  echo "[run_all] referee_listener not built or not executable; skipping"
fi

# start the main binary
echo "[run_all] starting main: $BINARY $CONFIG"
mkdir -p "$ROOT_DIR/logs/run"
nohup "$ROOT_DIR/$BINARY" "$ROOT_DIR/$CONFIG" > "$LOGDIR/main.log" 2>&1 &
echo "[run_all] main PID $! (logging to $LOGDIR/main.log)"

echo "[run_all] All started. Check logs in $LOGDIR and candump output in candump.log"

echo "To stop: pkill -f candump || true; pkill -f referee_listener || true; pkill -f $(basename "$BINARY") || true"
