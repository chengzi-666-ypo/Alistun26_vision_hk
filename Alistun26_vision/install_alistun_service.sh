#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="alistun_vision.service"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR/Alistun25_vision_hk"
PROGRAM="$PROJECT_DIR/build/auto_aim_test"

if [[ ! -x "$PROGRAM" ]]; then
    echo "错误：找不到可执行主程序：$PROGRAM" >&2
    echo "请先确认项目已完成编译。" >&2
    exit 1
fi

TARGET_USER="${SUDO_USER:-${USER}}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"

if [[ -z "$TARGET_HOME" ]]; then
    echo "错误：无法确定用户 $TARGET_USER 的主目录。" >&2
    exit 1
fi

generate_unit() {
    cat <<EOF
[Unit]
Description=Alistun25 Vision Autostart Service
After=graphical.target network.target
Wants=graphical.target

[Service]
Type=simple
User=$TARGET_USER
WorkingDirectory=$PROJECT_DIR
Environment=HOME=$TARGET_HOME
Environment=DISPLAY=:0
Environment=XAUTHORITY=$TARGET_HOME/.Xauthority
ExecStartPre=/bin/sleep 5
ExecStart="$PROGRAM"
Restart=on-failure
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=graphical.target
EOF
}

if [[ "${1:-}" == "--dry-run" ]]; then
    generate_unit
    exit 0
fi

if [[ "$EUID" -ne 0 ]]; then
    echo "请使用 sudo 运行安装脚本："
    echo "  sudo \"$0\""
    exit 1
fi

UNIT_PATH="/etc/systemd/system/$SERVICE_NAME"
TMP_FILE="$(mktemp)"
trap 'rm -f "$TMP_FILE"' EXIT

generate_unit > "$TMP_FILE"
chmod 0644 "$TMP_FILE"
install -m 0644 "$TMP_FILE" "$UNIT_PATH"

systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"

echo
echo "$SERVICE_NAME 已安装、启用并启动。"
echo "查看状态：systemctl status $SERVICE_NAME --no-pager"
echo "查看日志：journalctl -u $SERVICE_NAME -f"
