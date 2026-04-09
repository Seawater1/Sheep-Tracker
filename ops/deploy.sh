#!/usr/bin/env bash

set -euo pipefail

source "$(cd "$(dirname "$0")" && pwd)/lib.sh"
load_env
require_env VPS_APP_DIR VPS_SERVER_DIR VPS_DASHBOARD_DIR
APP_RUNTIME_USER="${VPS_RUNTIME_USER:-root}"
SUDO="$(remote_sudo_prefix)"
VENV_DIR="${VPS_APP_DIR}/.venv"

ssh_cmd "mkdir -p '${VPS_SERVER_DIR}' '${VPS_DASHBOARD_DIR}'"

scp_to_vps \
  "${ROOT_DIR}/server/tracker_api.py" \
  "${ROOT_DIR}/server/README.md" \
  "${VPS_USER}@${VPS_HOST}:${VPS_SERVER_DIR}/"

scp_to_vps \
  "${ROOT_DIR}/dashboard/index.html" \
  "${ROOT_DIR}/dashboard/styles.css" \
  "${ROOT_DIR}/dashboard/app.js" \
  "${ROOT_DIR}/dashboard/sample-event.js" \
  "${ROOT_DIR}/dashboard/README.md" \
  "${VPS_USER}@${VPS_HOST}:${VPS_DASHBOARD_DIR}/"

ssh_cmd "python3 -m venv '${VENV_DIR}'"
ssh_cmd "'${VENV_DIR}/bin/python' -m py_compile '${VPS_SERVER_DIR}/tracker_api.py'"
ssh_cmd "${SUDO}chown -R '${APP_RUNTIME_USER}':'${APP_RUNTIME_USER}' '${VPS_APP_DIR}'"

ssh_cmd "cat > /tmp/sheep-tracker.service <<'EOF'
[Unit]
Description=Sheep Tracker TTN Bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=${VPS_APP_DIR}
Environment=SHEEP_TRACKER_PORT=8781
Environment=SHEEP_TRACKER_DB=${VPS_SERVER_DIR}/tracker.db
Environment=SHEEP_TRACKER_STATIC_DIR=${VPS_DASHBOARD_DIR}
ExecStart=${VENV_DIR}/bin/python ${VPS_SERVER_DIR}/tracker_api.py
Restart=always
RestartSec=5
User=${APP_RUNTIME_USER}

[Install]
WantedBy=multi-user.target
EOF
${SUDO}mv /tmp/sheep-tracker.service /etc/systemd/system/sheep-tracker.service
${SUDO}systemctl daemon-reload
${SUDO}systemctl enable sheep-tracker.service
${SUDO}systemctl restart sheep-tracker.service"

echo "Deploy complete."
