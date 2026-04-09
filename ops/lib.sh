#!/usr/bin/env bash

set -euo pipefail

OPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${OPS_DIR}/.." && pwd)"
ENV_FILE="${OPS_DIR}/.env.local"

require_env() {
  local missing=0
  for key in "$@"; do
    if [[ -z "${!key:-}" ]]; then
      echo "Missing required env var: ${key}" >&2
      missing=1
    fi
  done
  if [[ "${missing}" -ne 0 ]]; then
    exit 1
  fi
}

load_env() {
  if [[ ! -f "${ENV_FILE}" ]]; then
    echo "Missing ${ENV_FILE}. Copy ops/.env.example to ops/.env.local first." >&2
    exit 1
  fi
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
}

ssh_cmd() {
  require_env VPS_HOST VPS_USER VPS_SSH_KEY_PATH
  local port="${VPS_SSH_PORT:-22}"
  ssh -i "${VPS_SSH_KEY_PATH}" -p "${port}" \
    -o BatchMode=yes \
    -o StrictHostKeyChecking=accept-new \
    "${VPS_USER}@${VPS_HOST}" "$@"
}

scp_to_vps() {
  require_env VPS_HOST VPS_USER VPS_SSH_KEY_PATH
  local port="${VPS_SSH_PORT:-22}"
  scp -i "${VPS_SSH_KEY_PATH}" -P "${port}" \
    -o StrictHostKeyChecking=accept-new \
    "$@"
}

remote_sudo_prefix() {
  if [[ "${VPS_USER:-}" == "root" ]]; then
    printf '%s' ""
  else
    printf '%s' "sudo "
  fi
}
