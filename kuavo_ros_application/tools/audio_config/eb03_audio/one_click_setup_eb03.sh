#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_USER="${TARGET_USER:-${SUDO_USER:-$(id -un)}}"
TARGET_UID="$(id -u "${TARGET_USER}")"
TARGET_HOME="$(eval echo "~${TARGET_USER}")"

AUDIO_DIR="${TARGET_HOME}/audio"
USER_SYSTEMD_DIR="${TARGET_HOME}/.config/systemd/user"
USER_PULSE_DIR="${TARGET_HOME}/.config/pulse"
USER_AUTOSTART_DIR="${TARGET_HOME}/.config/autostart"

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "[ERROR] Required file not found: ${path}" >&2
    exit 1
  fi
}

copy_file_if_needed() {
  local src="$1"
  local dst="$2"
  local mode="$3"

  if [[ "$(realpath "${src}")" == "$(realpath -m "${dst}")" ]]; then
    chmod "${mode}" "${dst}" 2>/dev/null || true
    return 0
  fi

  install -m "${mode}" "${src}" "${dst}"
}

render_service_with_execstart() {
  local src="$1"
  local dst="$2"
  local mode="$3"

  python3 - <<PY
from pathlib import Path

src = Path(r"${src}")
dst = Path(r"${dst}")
installed_script = Path(r"${AUDIO_DIR}") / "set_pulse_default_eb03.sh"
text = src.read_text(encoding="utf-8", errors="ignore")
lines = text.replace("\r\n", "\n").replace("\r", "\n").splitlines()
for i, line in enumerate(lines):
    if line.startswith("ExecStart="):
        lines[i] = f"ExecStart=/bin/bash {installed_script} EB03_LJ"
        break
text = "\n".join(lines) + "\n"
with dst.open("w", encoding="utf-8", newline="\n") as f:
    f.write(text)
PY

  chmod "${mode}" "${dst}"
}

write_hidden_autostart() {
  mkdir -p "${USER_AUTOSTART_DIR}"
  cat > "${USER_AUTOSTART_DIR}/pulseaudio.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=PulseAudio (disabled override)
Hidden=true
EOF
  chown "${TARGET_USER}:${TARGET_USER}" "${USER_AUTOSTART_DIR}/pulseaudio.desktop"
}

patch_default_pa() {
  local default_pa="${USER_PULSE_DIR}/default.pa"
  local backup_pa="${USER_PULSE_DIR}/default.pa.bak"

  mkdir -p "${USER_PULSE_DIR}"
  if [[ ! -f "${default_pa}" ]]; then
    cp /etc/pulse/default.pa "${default_pa}"
    chown "${TARGET_USER}:${TARGET_USER}" "${default_pa}"
  fi

  if [[ ! -f "${backup_pa}" ]]; then
    cp "${default_pa}" "${backup_pa}"
    chown "${TARGET_USER}:${TARGET_USER}" "${backup_pa}"
  fi

  python3 - <<PY
from pathlib import Path
p = Path(r"${default_pa}")
text = p.read_text(encoding="utf-8", errors="ignore")
text = text.replace("load-module module-udev-detect\n", "load-module module-udev-detect use_ucm=0\n")
text = text.replace("load-module module-switch-on-port-available\n", "# disabled by eb03 fix: load-module module-switch-on-port-available\n")
text = text.replace("load-module module-switch-on-connect\n", "# disabled by eb03 fix: load-module module-switch-on-connect\n")
text = text.replace("load-module module-default-device-restore\n", "# disabled by eb03 fix: load-module module-default-device-restore\n")
p.write_text(text, encoding="utf-8")
PY
  chown "${TARGET_USER}:${TARGET_USER}" "${default_pa}"
}

write_client_conf() {
  mkdir -p "${USER_PULSE_DIR}"
  printf '%s\n' 'autospawn = no' > "${USER_PULSE_DIR}/client.conf"
  chown "${TARGET_USER}:${TARGET_USER}" "${USER_PULSE_DIR}/client.conf"
}

cleanup_conflicting_pulseaudio() {
  pkill -u 0 -x pulseaudio >/dev/null 2>&1 || true
  pkill -u gdm -x pulseaudio >/dev/null 2>&1 || true
  sudo -u "${TARGET_USER}" pulseaudio -k >/dev/null 2>&1 || true
  pkill -u "${TARGET_USER}" -x pulseaudio >/dev/null 2>&1 || true
  rm -rf "/run/user/${TARGET_UID}/pulse" >/dev/null 2>&1 || true
  sleep 1
}

enable_user_units() {
  sudo -u "${TARGET_USER}" \
    XDG_RUNTIME_DIR="/run/user/${TARGET_UID}" \
    DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/${TARGET_UID}/bus" \
    systemctl --user daemon-reload

  sudo -u "${TARGET_USER}" \
    XDG_RUNTIME_DIR="/run/user/${TARGET_UID}" \
    DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/${TARGET_UID}/bus" \
    systemctl --user enable --now pulseaudio.socket set-pulse-default-eb03.timer
}

run_fix_now() {
  sudo -u "${TARGET_USER}" \
    TARGET_USER="${TARGET_USER}" \
    XDG_RUNTIME_DIR="/run/user/${TARGET_UID}" \
    DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/${TARGET_UID}/bus" \
    /bin/bash "${SCRIPT_DIR}/set_pulse_default_eb03.sh"
}

if [[ "${EUID}" -ne 0 ]]; then
  echo "[ERROR] Please run as root or with sudo." >&2
  exit 1
fi

if ! id "${TARGET_USER}" >/dev/null 2>&1; then
  echo "[ERROR] Target user does not exist: ${TARGET_USER}" >&2
  exit 1
fi

require_file "${SCRIPT_DIR}/set_pulse_default_eb03.sh"
require_file "${SCRIPT_DIR}/set-pulse-default-eb03.service"
require_file "${SCRIPT_DIR}/set-pulse-default-eb03.timer"
require_file "${SCRIPT_DIR}/README_audio_setup.md"

echo "[1/8] Creating target directories"
mkdir -p "${AUDIO_DIR}" "${USER_SYSTEMD_DIR}" "${USER_PULSE_DIR}" "${USER_AUTOSTART_DIR}"
chown -R "${TARGET_USER}:${TARGET_USER}" "${AUDIO_DIR}" "${USER_SYSTEMD_DIR}" "${USER_PULSE_DIR}" "${USER_AUTOSTART_DIR}"

echo "[2/8] Copying scripts and documents"
copy_file_if_needed "${SCRIPT_DIR}/set_pulse_default_eb03.sh" "${AUDIO_DIR}/set_pulse_default_eb03.sh" 755
copy_file_if_needed "${SCRIPT_DIR}/README_audio_setup.md" "${AUDIO_DIR}/README_audio_setup.md" 644

echo "[3/8] Installing user systemd units"
render_service_with_execstart "${SCRIPT_DIR}/set-pulse-default-eb03.service" "${USER_SYSTEMD_DIR}/set-pulse-default-eb03.service" 644
copy_file_if_needed "${SCRIPT_DIR}/set-pulse-default-eb03.timer" "${USER_SYSTEMD_DIR}/set-pulse-default-eb03.timer" 644
python3 - <<PY
from pathlib import Path
for p in [
    Path(r"${AUDIO_DIR}/set_pulse_default_eb03.sh"),
    Path(r"${AUDIO_DIR}/README_audio_setup.md"),
    Path(r"${USER_SYSTEMD_DIR}/set-pulse-default-eb03.service"),
    Path(r"${USER_SYSTEMD_DIR}/set-pulse-default-eb03.timer"),
]:
    p.write_bytes(p.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n"))
PY
chown -R "${TARGET_USER}:${TARGET_USER}" "${AUDIO_DIR}" "${USER_SYSTEMD_DIR}"

echo "[4/8] Disabling conflicting PulseAudio autostart paths"
write_client_conf
write_hidden_autostart

echo "[5/8] Patching user PulseAudio config"
patch_default_pa

echo "[6/8] Enabling linger and user PulseAudio socket"
loginctl enable-linger "${TARGET_USER}" >/dev/null 2>&1 || true

echo "[7/9] Cleaning conflicting PulseAudio processes"
cleanup_conflicting_pulseaudio

echo "[8/9] Enabling user services"
enable_user_units

echo "[9/9] Running recovery script now"
run_fix_now || true

echo
echo "One-click setup complete."
echo "Target user: ${TARGET_USER}"
echo "Audio directory: ${AUDIO_DIR}"
echo
echo "Recommended verification:"
echo "  sudo -u ${TARGET_USER} XDG_RUNTIME_DIR=/run/user/${TARGET_UID} DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/${TARGET_UID}/bus pactl info"
echo "  sudo -u ${TARGET_USER} XDG_RUNTIME_DIR=/run/user/${TARGET_UID} DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/${TARGET_UID}/bus pactl list short sinks"
echo "  sudo -u ${TARGET_USER} XDG_RUNTIME_DIR=/run/user/${TARGET_UID} DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/${TARGET_UID}/bus pactl list short sources"
