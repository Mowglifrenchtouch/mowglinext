#!/bin/bash
# =============================================================================
# Send a one-time UM98x rover configuration over the serial port.
#
# Symptoms this fixes:
#   * accuracy stuck at 10 m, RTK never validates, even with RTCM flowing
#   * /diagnostics carr_soln stays "none"
#
# Root cause: out-of-the-box / FRESET-state UM98x receivers default to
# something close to "rover, no NMEA enabled, no PVTSLNA" — and the
# default RTK timeout is short. Without an explicit MODE ROVER + LOG
# directives, the receiver outputs only minimal NMEA and the operator
# sees a single-point fix even though RTCM is being injected.
#
# We send commands using the port-agnostic short form
# (`LOG <msg> ONTIME <rate>`) so the receiver outputs on whichever
# physical UART the host is connected to.
#
# Reference: https://github.com/CentipedeRTK/docs-centipedeRTK
#            (assets/param_files/UM98x/UM980_aog_rover_last_CONFIG.txt)
#
# Usage: configure_receiver.sh [device] [baudrate]
#
# Persisted via SAVECONFIG in receiver NVRAM, so this only meaningfully
# changes anything on the first run; subsequent calls are idempotent.
# =============================================================================
set -euo pipefail

# NOTE:
# The serial device path (usually /dev/gps) is managed by Linux udev rules
# and must remain stable across all GNSS backends.
#
# DEVICE is the Linux serial path, normally /dev/gps.
# UNICORE_COM_PORT is ONLY the internal receiver-side logical COM port
# used in commands such as:
#   CONFIG COM1 921600
#
# Do NOT derive the Linux serial device path from UNICORE_COM_PORT.
TARGET_BAUD="${UNICORE_TARGET_BAUD:-921600}"
UNICORE_COM_PORT="${UNICORE_COM_PORT:-COM1}"
UNICORE_PROFILE="${UNICORE_PROFILE:-normal}"
UNICORE_SIGNALGROUP_OVERRIDE="${UNICORE_SIGNALGROUP_OVERRIDE:-}"
UNICORE_MAIN_LOG_PERIOD="${UNICORE_MAIN_LOG_PERIOD:-}"
UNICORE_BESTNAV_LOG_PERIOD="${UNICORE_BESTNAV_LOG_PERIOD:-}"
UNICORE_DIAGNOSTIC_LOG_PERIOD="${UNICORE_DIAGNOSTIC_LOG_PERIOD:-}"
UNICORE_SATELLITE_LOG_PERIOD="${UNICORE_SATELLITE_LOG_PERIOD:-}"
UNICORE_RF_LOG_PERIOD="${UNICORE_RF_LOG_PERIOD:-}"
UNICORE_RAW_LOG_PERIOD="${UNICORE_RAW_LOG_PERIOD:-}"
UNICORE_ENABLE_SATELLITES="${UNICORE_ENABLE_SATELLITES:-}"
UNICORE_ENABLE_RF="${UNICORE_ENABLE_RF:-}"
UNICORE_ENABLE_JAMMING="${UNICORE_ENABLE_JAMMING:-}"
UNICORE_ENABLE_HARDWARE="${UNICORE_ENABLE_HARDWARE:-}"

log() {
  echo "[configure_receiver.sh] $*" >&2
}

unicore_is_truthy() {
  case "${1,,}" in
    true|1|yes|y|on)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

unicore_bool_string() {
  if unicore_is_truthy "${1:-false}"; then
    printf '%s\n' "true"
  else
    printf '%s\n' "false"
  fi
}

unicore_normalize_profile() {
  case "${1:-normal}" in
    normal|debug|survey|high_precision)
      printf '%s\n' "${1:-normal}"
      ;;
    *)
      log "WARN: unknown UNICORE_PROFILE='${1:-}'; falling back to normal."
      printf '%s\n' "normal"
      ;;
  esac
}

unicore_profile_default_period() {
  local profile="${1:?unicore_profile_default_period: missing profile}"
  local kind="${2:?unicore_profile_default_period: missing kind}"

  case "$profile:$kind" in
    normal:main) printf '%s\n' "0.2" ;;
    normal:bestnav) printf '%s\n' "0.2" ;;
    normal:diagnostic) printf '%s\n' "1" ;;
    normal:satellite) printf '%s\n' "1" ;;
    normal:rf) printf '%s\n' "1" ;;
    normal:raw) printf '%s\n' "5" ;;
    debug:main) printf '%s\n' "0.2" ;;
    debug:bestnav) printf '%s\n' "0.2" ;;
    debug:diagnostic) printf '%s\n' "1" ;;
    debug:satellite) printf '%s\n' "1" ;;
    debug:rf) printf '%s\n' "1" ;;
    debug:raw) printf '%s\n' "5" ;;
    survey:main) printf '%s\n' "1" ;;
    survey:bestnav) printf '%s\n' "1" ;;
    survey:diagnostic) printf '%s\n' "1" ;;
    survey:satellite) printf '%s\n' "2" ;;
    survey:rf) printf '%s\n' "2" ;;
    survey:raw) printf '%s\n' "5" ;;
    high_precision:main) printf '%s\n' "0.1" ;;
    high_precision:bestnav) printf '%s\n' "0.1" ;;
    high_precision:diagnostic) printf '%s\n' "1" ;;
    high_precision:satellite) printf '%s\n' "1" ;;
    high_precision:rf) printf '%s\n' "1" ;;
    high_precision:raw) printf '%s\n' "5" ;;
    *)
      return 1
      ;;
  esac
}

unicore_profile_default_flag() {
  local profile="${1:?unicore_profile_default_flag: missing profile}"
  local feature="${2:?unicore_profile_default_flag: missing feature}"

  case "$profile:$feature" in
    normal:satellites|normal:rf|normal:jamming|normal:hardware)
      printf '%s\n' "false"
      ;;
    debug:satellites|debug:rf|debug:jamming|debug:hardware)
      printf '%s\n' "true"
      ;;
    survey:satellites|survey:rf|survey:jamming|survey:hardware)
      printf '%s\n' "true"
      ;;
    high_precision:satellites|high_precision:rf|high_precision:jamming|high_precision:hardware)
      printf '%s\n' "true"
      ;;
    *)
      return 1
      ;;
  esac
}

unicore_apply_profile_defaults() {
  UNICORE_PROFILE="$(unicore_normalize_profile "$UNICORE_PROFILE")"
  UNICORE_MAIN_LOG_PERIOD="${UNICORE_MAIN_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" main)}"
  UNICORE_BESTNAV_LOG_PERIOD="${UNICORE_BESTNAV_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" bestnav)}"
  UNICORE_DIAGNOSTIC_LOG_PERIOD="${UNICORE_DIAGNOSTIC_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" diagnostic)}"
  UNICORE_SATELLITE_LOG_PERIOD="${UNICORE_SATELLITE_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" satellite)}"
  UNICORE_RF_LOG_PERIOD="${UNICORE_RF_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" rf)}"
  UNICORE_RAW_LOG_PERIOD="${UNICORE_RAW_LOG_PERIOD:-$(unicore_profile_default_period "$UNICORE_PROFILE" raw)}"
  UNICORE_ENABLE_SATELLITES="${UNICORE_ENABLE_SATELLITES:-$(unicore_profile_default_flag "$UNICORE_PROFILE" satellites)}"
  UNICORE_ENABLE_RF="${UNICORE_ENABLE_RF:-$(unicore_profile_default_flag "$UNICORE_PROFILE" rf)}"
  UNICORE_ENABLE_JAMMING="${UNICORE_ENABLE_JAMMING:-$(unicore_profile_default_flag "$UNICORE_PROFILE" jamming)}"
  UNICORE_ENABLE_HARDWARE="${UNICORE_ENABLE_HARDWARE:-$(unicore_profile_default_flag "$UNICORE_PROFILE" hardware)}"
}

require_serial_port() {
  local port="${1:?require_serial_port: missing port}"

  if [ ! -c "$port" ]; then
    log "ERROR: $port is not a character device" >&2
    return 1
  fi
}

wait_for_serial_port() {
  local port="${1:?wait_for_serial_port: missing port}"
  local attempt

  for attempt in $(seq 1 20); do
    [ -w "$port" ] && return 0
    sleep 0.1
  done

  log "ERROR: $port is not writable after waiting" >&2
  return 1
}

serial_set_baud() {
  local port="${1:?serial_set_baud: missing port}"
  local baud="${2:?serial_set_baud: missing baud}"

  stty -F "$port" "$baud" raw -echo -echoe -echok -ixon -ixoff || {
    log "ERROR: stty failed on $port @ $baud" >&2
    return 1
  }
}

open_serial() {
  local port="${1:?open_serial: missing port}"

  exec 3<>"$port"
}

close_serial() {
  exec 3>&- || true
}

drain_serial() {
  local line

  while IFS= read -r -t 0.05 -u 3 line; do
    :
  done
}

send_serial_command() {
  local command="${1:?send_serial_command: missing command}"

  printf '%s\r\n' "$command" >&3
}

collect_serial_output() {
  local rounds="${1:?collect_serial_output: missing rounds}"
  local line output=""
  local i

  for i in $(seq 1 "$rounds"); do
    if IFS= read -r -t 0.15 -u 3 line; then
      output+="${line}"$'\n'
    fi
  done

  printf '%s' "$output"
}

query_receiver_identification() {
  local response=""

  drain_serial

  # `VERSION` is the documented query command; `VERSIONA` is a compatible
  # fallback on receivers that accept the log/message name directly.
  send_serial_command "VERSION"
  sleep 0.1
  response="$(collect_serial_output 10)"

  if [[ "$response" != *"UM980"* && "$response" != *"UM982"* && "$response" != *"#VERSIONA"* ]]; then
    send_serial_command "VERSIONA"
    sleep 0.1
    response+=$(collect_serial_output 10)
  fi

  printf '%s' "$response"
}

model_from_response() {
  local response="${1:-}"

  if [[ "$response" == *"UM980"* ]]; then
    printf '%s\n' "UM980"
  elif [[ "$response" == *"UM982"* ]]; then
    printf '%s\n' "UM982"
  else
    printf '%s\n' "unknown"
  fi
}

detect_receiver_at_baud() {
  local port="${1:?detect_receiver_at_baud: missing port}"
  local baud="${2:?detect_receiver_at_baud: missing baud}"
  local response model

  serial_set_baud "$port" "$baud" || return 1
  open_serial "$port"
  response="$(query_receiver_identification)"
  close_serial

  if [ -z "$response" ]; then
    return 1
  fi

  model="$(model_from_response "$response")"
  printf '%s|%s\n' "$baud" "$model"
}

detect_baud_candidates() {
  local preferred="${1:-}"
  local seen=" "
  local candidate
  local ordered=()

  for candidate in "$preferred" "$TARGET_BAUD" 460800 115200 230400 57600 38400 9600; do
    [ -n "$candidate" ] || continue
    case "$seen" in
      *" $candidate "*) continue ;;
    esac
    ordered+=("$candidate")
    seen+="$(printf '%s ' "$candidate")"
  done

  printf '%s\n' "${ordered[@]}"
}

detect_receiver_baud_and_model() {
  local port="${1:?detect_receiver_baud_and_model: missing port}"
  local preferred_baud="${2:-}"
  local candidate result baud model

  while IFS= read -r candidate; do
    [ -n "$candidate" ] || continue
    log "Probing receiver on ${port} @ ${candidate}..."

    if result="$(detect_receiver_at_baud "$port" "$candidate")"; then
      baud="${result%%|*}"
      model="${result#*|}"
      log "Receiver responded on ${baud}."
      printf '%s|%s\n' "$baud" "$model"
      return 0
    fi
  done < <(detect_baud_candidates "$preferred_baud")

  return 1
}

signalgroup_for_model() {
  local model="${1:?signalgroup_for_model: missing model}"

  if [ -n "$UNICORE_SIGNALGROUP_OVERRIDE" ]; then
    printf '%s\n' "$UNICORE_SIGNALGROUP_OVERRIDE"
    return 0
  fi

  case "$model" in
    UM980) printf '%s\n' "CONFIG SIGNALGROUP 2" ;;
    UM982) printf '%s\n' "CONFIG SIGNALGROUP 3 6" ;;
    *) return 1 ;;
  esac
}

build_log_commands() {
  printf '%s\n' "LOG GPGGA ONTIME ${UNICORE_MAIN_LOG_PERIOD}"
  printf '%s\n' "LOG PVTSLNA ONTIME ${UNICORE_MAIN_LOG_PERIOD}"
  printf '%s\n' "LOG BESTNAVA ONTIME ${UNICORE_BESTNAV_LOG_PERIOD}"
  printf '%s\n' "LOG GNHPR ONTIME ${UNICORE_MAIN_LOG_PERIOD}"
  printf '%s\n' "LOG RTKSTATUSA ONTIME ${UNICORE_DIAGNOSTIC_LOG_PERIOD}"
  printf '%s\n' "LOG RTCMSTATUSA ONTIME ${UNICORE_DIAGNOSTIC_LOG_PERIOD}"

  if unicore_is_truthy "$UNICORE_ENABLE_SATELLITES"; then
    printf '%s\n' "LOG BESTSATA ONTIME ${UNICORE_SATELLITE_LOG_PERIOD}"
    printf '%s\n' "LOG SATSINFOA ONTIME ${UNICORE_SATELLITE_LOG_PERIOD}"
    printf '%s\n' "LOG GPGSV ONTIME ${UNICORE_SATELLITE_LOG_PERIOD}"
    printf '%s\n' "LOG GLGSV ONTIME ${UNICORE_SATELLITE_LOG_PERIOD}"
    printf '%s\n' "LOG GAGSV ONTIME ${UNICORE_SATELLITE_LOG_PERIOD}"
    printf '%s\n' "LOG GBGSV ONTIME ${UNICORE_SATELLITE_LOG_PERIOD}"
  fi

  if unicore_is_truthy "$UNICORE_ENABLE_RF"; then
    printf '%s\n' "LOG AGCA ONTIME ${UNICORE_RF_LOG_PERIOD}"
  fi

  if unicore_is_truthy "$UNICORE_ENABLE_HARDWARE"; then
    printf '%s\n' "LOG HWSTATUSA ONTIME ${UNICORE_RF_LOG_PERIOD}"
  fi

  if unicore_is_truthy "$UNICORE_ENABLE_JAMMING"; then
    printf '%s\n' "LOG JAMSTATUSA ONTIME ${UNICORE_RF_LOG_PERIOD}"
    printf '%s\n' "LOG FREQJAMSTATUSA ONTIME ${UNICORE_RF_LOG_PERIOD}"
  fi

  if [ "$UNICORE_PROFILE" = "survey" ]; then
    # OBSVMCMPA is ASCII and much more compact than OBSVMA, but it is not
    # parsed by the current driver yet. Keep it survey-only and slow.
    printf '%s\n' "LOG OBSVMCMPA ONTIME ${UNICORE_RAW_LOG_PERIOD}"
  fi
}

build_profile_commands() {
  case "$UNICORE_PROFILE" in
    high_precision)
      # These commands are documented in N4 R1.4 but may still depend on
      # firmware generation. Keep them profile-scoped rather than default.
      printf '%s\n' "CONFIG PVTALG MULTI"
      printf '%s\n' "CONFIG RTCMDECAUTO ENABLE"
      printf '%s\n' "CONFIG RTCMPHASERATE POSITIVE"
      printf '%s\n' "CONFIG RTCMCLOCKOFFSET ENABLE"
      ;;
    *)
      ;;
  esac
}

verify_receiver_at_baud() {
  local port="${1:?verify_receiver_at_baud: missing port}"
  local baud="${2:?verify_receiver_at_baud: missing baud}"
  local response

  serial_set_baud "$port" "$baud" || return 1
  open_serial "$port"
  response="$(query_receiver_identification)"
  close_serial

  [ -n "$response" ]
}

send_config_batch() {
  local port="${1:?send_config_batch: missing port}"
  shift
  local commands=("$@")
  local command

  serial_set_baud "$port" "$TARGET_BAUD"
  open_serial "$port"
  drain_serial

  for command in "${commands[@]}"; do
    send_serial_command "$command"
    log "  -> $command"
    sleep 0.2
  done

  close_serial
}

apply_receiver_configuration() {
  local port="${1:?apply_receiver_configuration: missing port}"
  local detected_baud="${2:?apply_receiver_configuration: missing detected baud}"
  local model="${3:?apply_receiver_configuration: missing model}"
  local signalgroup
  local cmds=()
  local log_cmds=()
  local profile_cmds=()

  unicore_apply_profile_defaults

  signalgroup="$(signalgroup_for_model "$model")" || {
    log "WARN: model '${model}' not clearly identified; refusing to apply SIGNALGROUP." >&2
    return 1
  }

  if [ "$detected_baud" != "$TARGET_BAUD" ]; then
    log "Switching ${UNICORE_COM_PORT} from ${detected_baud} to ${TARGET_BAUD}..."
    serial_set_baud "$port" "$detected_baud"
    open_serial "$port"
    send_serial_command "CONFIG ${UNICORE_COM_PORT} ${TARGET_BAUD}"
    close_serial
    sleep 0.5
  else
    log "Receiver already running at ${TARGET_BAUD}."
  fi

  if ! verify_receiver_at_baud "$port" "$TARGET_BAUD"; then
    log "ERROR: receiver did not answer after switch to ${TARGET_BAUD}." >&2
    return 1
  fi

  log "Receiver verified at ${TARGET_BAUD}; detected model ${model}."

  mapfile -t log_cmds < <(build_log_commands)
  mapfile -t profile_cmds < <(build_profile_commands)

  cmds=(
    # Rover mode + NMEA version + RTK timeouts.
    # SURVEY MOW = low-dynamics survey-grade rover with the mowing-specific
    # dynamics preset. UAV (which we tried first) is for drones — assumes
    # high vertical accelerations and hurts ambiguity resolution on a
    # ground robot that mostly translates. AUTOMOTIVE is the next-best
    # alternative if SURVEY MOW isn't supported on a given firmware rev.
  #  "FRESET"
    "MODE ROVER SURVEY MOW"
    "CONFIG NMEAVERSION V411"
    # 180 s tolerates short NTRIP outages without dropping back to single
    # point. Default is 60 s on most firmware revisions.
    "CONFIG RTK TIMEOUT 10"
#    "CONFIG RTK MMPL 1"
    "CONFIG RTK RELIABILITY 3 1"
    "CONFIG DGPS TIMEOUT 600"
    "CONFIG UNDULATION AUTO"
    "${signalgroup}"
    "CONFIG SBAS DISABLE"
    "CONFIG AGNSS DISABLE"
#    "CONFIG PVTALG MULTI"
#    "CONFIG MMP ENABLE"
#    "CONFIG PSRVELDRPOS ENABLE"
#    "CONFIG PPP DATUM WGS84"
#    "CONFIG ANTIJAM AUTO"
    "CONFIG PPS DISABLE"

    # Constellation enables. Mowgli mowers are typically in mid-latitude
    # open sky, so we want everything except QZSS (regional, doesn't help
    # in EU/NA).
    "MASK 10"
    "UNMASK GPS"
    "UNMASK GLO"
    "UNMASK GAL"
    "UNMASK BDS"
    "MASK QZSS"
    "MASK IRNSS"
    # Drop previously persisted output schedule on the current port so a
    # transition from debug/survey back to normal actually becomes lighter.
    "UNLOG"
  )
  cmds+=( "${profile_cmds[@]}" )
  cmds+=( "${log_cmds[@]}" )

  log "Applying UM98x rover config to ${port} @ ${TARGET_BAUD}..."
  log "Profile=${UNICORE_PROFILE} main=${UNICORE_MAIN_LOG_PERIOD}s bestnav=${UNICORE_BESTNAV_LOG_PERIOD}s diag=${UNICORE_DIAGNOSTIC_LOG_PERIOD}s sat=${UNICORE_SATELLITE_LOG_PERIOD}s rf=${UNICORE_RF_LOG_PERIOD}s raw=${UNICORE_RAW_LOG_PERIOD}s sat_enabled=$(unicore_bool_string "$UNICORE_ENABLE_SATELLITES") rf_enabled=$(unicore_bool_string "$UNICORE_ENABLE_RF") jam_enabled=$(unicore_bool_string "$UNICORE_ENABLE_JAMMING")"
  send_config_batch "$port" "${cmds[@]}"

  if ! verify_receiver_at_baud "$port" "$TARGET_BAUD"; then
    log "ERROR: receiver stopped responding at ${TARGET_BAUD} before SAVECONFIG." >&2
    return 1
  fi

  send_config_batch "$port" "SAVECONFIG"
  log "Done. Receiver config persisted via SAVECONFIG."
}

main() {
  local port="${1:-/dev/gps}"
  local preferred_baud="${2:-$TARGET_BAUD}"
  local detection detected_baud detected_model

  require_serial_port "$port"
  wait_for_serial_port "$port"

  detection="$(detect_receiver_baud_and_model "$port" "$preferred_baud")" || {
    log "ERROR: unable to detect a responding Unicore receiver on ${port}." >&2
    return 1
  }

  detected_baud="${detection%%|*}"
  detected_model="${detection#*|}"

  if [ "$detected_model" = "unknown" ]; then
    log "WARN: receiver model could not be identified from VERSION response." >&2
    log "WARN: no SIGNALGROUP will be applied and SAVECONFIG is skipped." >&2
    return 1
  fi

  apply_receiver_configuration "$port" "$detected_baud" "$detected_model"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
