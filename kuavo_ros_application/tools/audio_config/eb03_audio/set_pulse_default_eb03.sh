#!/usr/bin/env bash

set -euo pipefail

TARGET_KEYWORD="${1:-EB03_LJ}"
TARGET_USER="${TARGET_USER:-${SUDO_USER:-$(id -un)}}"
TARGET_UID="$(id -u "${TARGET_USER}")"
WAIT_SECONDS="${WAIT_SECONDS:-0}"
RESTART_INTERVAL_SECONDS="${RESTART_INTERVAL_SECONDS:-15}"

USER_ID="${TARGET_UID}"
PULSE_DIR="/run/user/${USER_ID}/pulse"
PULSE_SOCKET="${PULSE_DIR}/native"
USER_BUS="unix:path=/run/user/${USER_ID}/bus"

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/${USER_ID}}"

run_pactl() {
  if [[ "$(id -un)" == "${TARGET_USER}" ]]; then
    pactl "$@"
  else
    sudo -u "${TARGET_USER}" \
      XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
      DBUS_SESSION_BUS_ADDRESS="${USER_BUS}" \
      pactl "$@"
  fi
}

run_systemctl_user() {
  if [[ "$(id -un)" == "${TARGET_USER}" ]]; then
    systemctl --user "$@"
  else
    sudo -u "${TARGET_USER}" \
      XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
      DBUS_SESSION_BUS_ADDRESS="${USER_BUS}" \
      systemctl --user "$@"
  fi
}

run_pulseaudio_start() {
  if [[ "$(id -un)" == "${TARGET_USER}" ]]; then
    pulseaudio --start --log-target=syslog
  else
    sudo -u "${TARGET_USER}" \
      XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
      DBUS_SESSION_BUS_ADDRESS="${USER_BUS}" \
      pulseaudio --start --log-target=syslog
  fi
}

run_pulseaudio_kill() {
  if [[ "$(id -un)" == "${TARGET_USER}" ]]; then
    pulseaudio --kill
  else
    sudo -u "${TARGET_USER}" \
      XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
      DBUS_SESSION_BUS_ADDRESS="${USER_BUS}" \
      pulseaudio --kill
  fi
}

restart_pulse_server() {
  echo "Restarting PulseAudio for ${TARGET_KEYWORD} re-detection" >&2

  if command -v systemctl >/dev/null 2>&1; then
    run_systemctl_user stop pulseaudio.service >/dev/null 2>&1 || true
    run_systemctl_user stop pulseaudio.socket >/dev/null 2>&1 || true
  fi

  run_pulseaudio_kill >/dev/null 2>&1 || true
  pkill -u "${TARGET_USER}" -x pulseaudio >/dev/null 2>&1 || true
  rm -rf "${PULSE_DIR:?}/"* >/dev/null 2>&1 || true
  sleep 1

  if command -v systemctl >/dev/null 2>&1; then
    run_systemctl_user start pulseaudio.socket >/dev/null 2>&1 || true
    run_systemctl_user start pulseaudio.service >/dev/null 2>&1 || true
  fi

  ensure_pulse_server
}

if ! command -v pactl >/dev/null 2>&1; then
  echo "pactl not found. Please install pulseaudio-utils or pipewire-pulse." >&2
  exit 1
fi

ensure_pulse_server() {
  if [[ ! -d "${XDG_RUNTIME_DIR}" ]]; then
    echo "XDG_RUNTIME_DIR does not exist: ${XDG_RUNTIME_DIR}" >&2
    exit 1
  fi

  mkdir -p "${PULSE_DIR}"

  if run_pactl info >/dev/null 2>&1; then
    return 0
  fi

  if command -v systemctl >/dev/null 2>&1; then
    run_systemctl_user start pulseaudio.socket >/dev/null 2>&1 || true
    run_systemctl_user start pulseaudio.service >/dev/null 2>&1 || true
    sleep 1
  fi

  if run_pactl info >/dev/null 2>&1; then
    return 0
  fi

  # If stale pulseaudio processes exist without a native socket, reset them first.
  if [[ ! -S "${PULSE_SOCKET}" ]]; then
    run_pulseaudio_kill >/dev/null 2>&1 || true
    pkill -u "${TARGET_USER}" -x pulseaudio >/dev/null 2>&1 || true
    rm -rf "${PULSE_DIR:?}/"* >/dev/null 2>&1 || true
    sleep 1
  fi

  # A preset PULSE_SERVER disables PulseAudio autospawn/startup.
  unset PULSE_SERVER
  if [[ "$(id -un)" == "${TARGET_USER}" ]]; then
    pulseaudio --check >/dev/null 2>&1 || run_pulseaudio_start >/dev/null 2>&1 || true
  else
    sudo -u "${TARGET_USER}" \
      XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
      DBUS_SESSION_BUS_ADDRESS="${USER_BUS}" \
      sh -c 'pulseaudio --check >/dev/null 2>&1 || pulseaudio --start --log-target=syslog >/dev/null 2>&1 || true'
  fi
  sleep 1

  if run_pactl info >/dev/null 2>&1; then
    return 0
  fi

  echo "Unable to connect to PulseAudio/PipeWire." >&2
  echo "Current PULSE_SERVER=${PULSE_SERVER-<unset>}" >&2
  if [[ -S "${PULSE_SOCKET}" ]]; then
    echo "Pulse socket exists: ${PULSE_SOCKET}" >&2
  else
    echo "Pulse socket missing: ${PULSE_SOCKET}" >&2
  fi
  exit 1
}

ensure_pulse_server

find_device() {
  local device_type="$1"
  local keyword="$2"

  run_pactl list short "${device_type}" | awk -v kw="$keyword" 'index($0, kw) { print $2; exit }'
}

find_real_source() {
  local keyword="$1"

  run_pactl list short sources | awk -v kw="$keyword" 'index($0, kw) && $2 !~ /\.monitor$/ { print $2; exit }'
}

source_exists() {
  local source_name="$1"
  run_pactl list short sources | awk -v name="$source_name" '$2 == name { found=1 } END { exit(found ? 0 : 1) }'
}

sink_exists() {
  local sink_name="$1"
  run_pactl list short sinks | awk -v name="$sink_name" '$2 == name { found=1 } END { exit(found ? 0 : 1) }'
}

card_exists() {
  local keyword="$1"
  run_pactl list cards short | awk -v kw="$keyword" 'index($0, kw) { found=1 } END { exit(found ? 0 : 1) }'
}

alsa_card_exists() {
  local keyword="$1"

  if [[ -r /proc/asound/cards ]] && grep -Fq "${keyword}" /proc/asound/cards; then
    return 0
  fi

  if command -v aplay >/dev/null 2>&1 && aplay -l 2>/dev/null | grep -Fq "${keyword}"; then
    return 0
  fi

  return 1
}

find_card_name() {
  local keyword="$1"
  run_pactl list cards short | awk -v kw="$keyword" 'index($0, kw) { print $2; exit }'
}

SINK_NAME=""
SOURCE_NAME=""
CARD_READY=0
ALSA_CARD_READY=0
WAIT_FOREVER=0
if [[ "${WAIT_SECONDS}" -le 0 ]]; then
  WAIT_FOREVER=1
fi

i=0
last_restart_at=0
while true; do
  sink_ready=0
  source_ready=0

  if alsa_card_exists "${TARGET_KEYWORD}"; then
    ALSA_CARD_READY=1
  else
    ALSA_CARD_READY=0
  fi

  if card_exists "${TARGET_KEYWORD}"; then
    CARD_READY=1
  else
    CARD_READY=0
  fi

  SINK_NAME="$(find_device sinks "${TARGET_KEYWORD}")"
  SOURCE_NAME="$(find_real_source "${TARGET_KEYWORD}")"

  if [[ -n "${SINK_NAME}" ]] && sink_exists "${SINK_NAME}"; then
    sink_ready=1
  fi

  if [[ -n "${SOURCE_NAME}" ]] && source_exists "${SOURCE_NAME}"; then
    source_ready=1
  fi

  if [[ "${CARD_READY}" -eq 1 && "${sink_ready}" -eq 1 && "${source_ready}" -eq 1 ]]; then
    break
  fi

  if [[ "${ALSA_CARD_READY}" -eq 1 && "${RESTART_INTERVAL_SECONDS}" -gt 0 ]]; then
    if [[ "${CARD_READY}" -eq 0 || "${sink_ready}" -eq 0 || "${source_ready}" -eq 0 ]]; then
      now="$(date +%s)"
      if [[ "${last_restart_at}" -eq 0 || $((now - last_restart_at)) -ge "${RESTART_INTERVAL_SECONDS}" ]]; then
        restart_pulse_server
        last_restart_at="${now}"
      fi
    fi
  fi

  i=$((i + 1))
  if [[ "${WAIT_FOREVER}" -eq 0 && "${i}" -ge "${WAIT_SECONDS}" ]]; then
    break
  fi

  sleep 1
done

if [[ "${ALSA_CARD_READY}" -ne 1 ]]; then
  echo "No ALSA sound card found containing keyword: ${TARGET_KEYWORD}" >&2
  if [[ -r /proc/asound/cards ]]; then
    cat /proc/asound/cards >&2
  fi
  exit 1
fi

if [[ "${CARD_READY}" -ne 1 ]]; then
  echo "ALSA card exists, but PulseAudio card not found for keyword: ${TARGET_KEYWORD}" >&2
  run_pactl list cards short >&2
  exit 1
fi

if [[ -z "${SINK_NAME}" ]] || ! sink_exists "${SINK_NAME}"; then
  echo "No usable sink found containing keyword: ${TARGET_KEYWORD}" >&2
  CARD_NAME="$(find_card_name "${TARGET_KEYWORD}")"
  if [[ -n "${CARD_NAME}" ]]; then
    echo "Matching PulseAudio card: ${CARD_NAME}" >&2
    run_pactl list cards | awk -v card="$CARD_NAME" '
      $0 ~ ("Name: " card) { print_block=1 }
      print_block { print }
      print_block && /^$/ { exit }
    ' >&2
  fi
  run_pactl list short sinks >&2
  exit 1
fi

if [[ -z "${SOURCE_NAME}" ]] || ! source_exists "${SOURCE_NAME}"; then
  echo "No usable source found containing keyword: ${TARGET_KEYWORD}" >&2
  CARD_NAME="$(find_card_name "${TARGET_KEYWORD}")"
  if [[ -n "${CARD_NAME}" ]]; then
    echo "Matching PulseAudio card: ${CARD_NAME}" >&2
    run_pactl list cards | awk -v card="$CARD_NAME" '
      $0 ~ ("Name: " card) { print_block=1 }
      print_block { print }
      print_block && /^$/ { exit }
    ' >&2
  fi
  run_pactl list short sources >&2
  exit 1
fi

run_pactl set-default-sink "${SINK_NAME}"
echo "Default sink set to: ${SINK_NAME}"

run_pactl set-default-source "${SOURCE_NAME}"
echo "Default source set to: ${SOURCE_NAME}"

echo
echo "Current pactl info:"
run_pactl info
