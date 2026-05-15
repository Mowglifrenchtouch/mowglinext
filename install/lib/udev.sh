#!/usr/bin/env bash

find_serial_by_id() {
  local by_id_dir="${SERIAL_BY_ID_DIR:-/dev/serial/by-id}"
  local pattern
  local match

  [ -d "$by_id_dir" ] || return 1

  for pattern in "$@"; do
    match="$(find "$by_id_dir" -maxdepth 1 -type l -iname "$pattern" | sort | head -n1)"
    if [ -n "$match" ]; then
      printf '%s\n' "$match"
      return 0
    fi
  done

  return 1
}

emit_by_id_udev_rule() {
  local by_id_path="$1"
  local symlink="$2"
  local resolved kernel vendor product serial line key value

  [ -L "$by_id_path" ] || return 1
  resolved="$(readlink -f "$by_id_path")"
  kernel="$(basename "$resolved")"
  [ -n "$kernel" ] || return 1

  vendor=""
  product=""
  serial=""

  # Prefer a USB-attribute match (idVendor/idProduct[/serial]) so the symlink
  # survives re-enumeration. The previous KERNEL=="ttyACMn" form broke as soon
  # as a device's CDC-ACM index changed (e.g. F9P resets after ublox_dgnss
  # writes config, jumping ttyACM1 -> ttyACM3, leaving /dev/gps dangling and
  # causing EIO on every RTCM write).
  if command -v udevadm >/dev/null 2>&1 && [ -e "$resolved" ]; then
    while IFS='=' read -r key value; do
      case "$key" in
        ID_VENDOR_ID)    vendor="$value" ;;
        ID_MODEL_ID)     product="$value" ;;
        ID_SERIAL_SHORT) serial="$value" ;;
      esac
    done < <(udevadm info --query=property --name="$resolved" 2>/dev/null)
  fi

  echo "# $symlink from $by_id_path"
  if [ -n "$vendor" ] && [ -n "$product" ]; then
    line="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${vendor}\", ATTRS{idProduct}==\"${product}\""
    # Pin by USB serial when one is exposed — protects against collisions with
    # other devices sharing the same VID/PID (e.g. generic 0483:5740 STM32 VCPs).
    [ -n "$serial" ] && line="${line}, ATTRS{serial}==\"${serial}\""
    line="${line}, SYMLINK+=\"${symlink}\", MODE=\"0666\""
    echo "$line"
  else
    # Fallback for environments where udevadm can't resolve USB attributes
    # (e.g. test sandboxes with fake device nodes). Kernel-name rules are not
    # stable across re-enumeration; warn callers via the comment above.
    echo "KERNEL==\"${kernel}\", SYMLINK+=\"${symlink}\", MODE=\"0666\""
  fi
}

emit_strict_gps_by_id_udev_rule() {
  local by_id_path="$1"
  local resolved kernel vendor product serial id_path id_path_tag line key value

  [ -L "$by_id_path" ] || return 1
  resolved="$(readlink -f "$by_id_path")"
  kernel="$(basename "$resolved")"
  [ -n "$kernel" ] || return 1

  vendor=""
  product=""
  serial=""
  id_path=""
  id_path_tag=""

  if ! command -v udevadm >/dev/null 2>&1 || [ ! -e "$resolved" ]; then
    error "Selected GPS device $by_id_path needs udevadm properties to build a unique /dev/gps rule. Refusing generic fallback."
    return 1
  fi

  while IFS='=' read -r key value; do
    case "$key" in
      ID_VENDOR_ID)  vendor="$value" ;;
      ID_MODEL_ID)   product="$value" ;;
      ID_SERIAL_SHORT) serial="$value" ;;
      ID_PATH)       id_path="$value" ;;
      ID_PATH_TAG)   id_path_tag="$value" ;;
    esac
  done < <(udevadm info --query=property --name="$resolved" 2>/dev/null)

  if [ -z "$vendor" ] || [ -z "$product" ]; then
    error "Selected GPS device $by_id_path is missing idVendor/idProduct in udevadm output. Refusing to generate /dev/gps rule."
    return 1
  fi

  echo "# gps from $by_id_path"
  line="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${vendor}\", ATTRS{idProduct}==\"${product}\""
  if [ -n "$serial" ]; then
    line="${line}, ATTRS{serial}==\"${serial}\""
  elif [ -n "$id_path" ]; then
    line="${line}, ENV{ID_PATH}==\"${id_path}\""
  elif [ -n "$id_path_tag" ]; then
    line="${line}, ENV{ID_PATH_TAG}==\"${id_path_tag}\""
  else
    error "Selected GPS device $by_id_path exposes only generic VID/PID ${vendor}:${product} with no serial or stable path selector. Refusing to generate non-unique /dev/gps rule."
    return 1
  fi

  line="${line}, SYMLINK+=\"gps\", MODE=\"0666\""
  echo "$line"
}

build_static_udev_rules() {
  cat <<'EOF'
# =========================================================
# Mowgli II - static udev rules
# =========================================================

# Mowgli STM32 board — match by product string to avoid confusion
# with other STM32 CDC devices (0483:5740 is generic STM32 VCP).
#SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE="0666"

# Known GPS USB devices
# NOTE: 0483:5740 (STM32 VCP) removed — it conflicts with the Mowgli board
# which uses the same vendor/product ID. If your GPS uses an STM32-based
# USB adapter, add a rule matching its specific product string instead.
#SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="gps", MODE="0666"
#SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01aa", SYMLINK+="gps", MODE="0666"
EOF
}

build_dynamic_udev_rules() {
  local by_id_path
  local gnss_backend
  local gps_protocol="${GPS_PROTOCOL:-UBX}"

  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"
  if [[ "${GNSS_BACKEND:-}" == "nmea" ]]; then
    gps_protocol="NMEA"
  fi

  echo "# ========================================================="
  echo "# Mowgli II - dynamic rules from current hardware selection"
  echo "# ========================================================="

  if by_id_path="$(find_serial_by_id "*Mowgli*")"; then
    emit_by_id_udev_rule "$by_id_path" "mowgli"
  fi

  # MAVROS uses the explicitly selected device. Prefer by-id when selected,
  # but keep the ttyACM/ttyUSB fallback for compatibility with manual choices.
  if [ "${HARDWARE_BACKEND:-mowgli}" = "mavros" ] && [ -n "${MAVROS_BY_ID:-}" ] && [ -e "${MAVROS_BY_ID}" ]; then
    if [ -L "$MAVROS_BY_ID" ]; then
      emit_by_id_udev_rule "$MAVROS_BY_ID" "mavros"
    else
      echo "KERNEL==\"$(basename "$MAVROS_BY_ID")\", SYMLINK+=\"mavros\", MODE=\"0666\""
    fi
  fi

  # GPS principal
  # Runtime expects /dev/gps only. Create it from the installer-selected
  # source device, whether that source was a USB by-id path or a UART tty.
  if [ "$gnss_backend" != "disabled" ]; then
    if [ "${GPS_CONNECTION:-usb}" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
      echo "KERNEL==\"$(basename "$GPS_UART_DEVICE")\", SYMLINK+=\"gps\", MODE=\"0666\""
    elif [ -n "${GPS_BY_ID:-}" ] && [ -e "${GPS_BY_ID}" ]; then
      emit_strict_gps_by_id_udev_rule "$GPS_BY_ID" || return 1
    fi
  fi

  # GPS debug (miniUART only)
  if [ "$gnss_backend" != "disabled" ] && [ "${GPS_DEBUG_ENABLED:-false}" = "true" ] && [ -n "${GPS_DEBUG_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$GPS_DEBUG_UART_DEVICE")\", SYMLINK+=\"gps_debug\", MODE=\"0666\""
  fi

  # LiDAR
  if [ "${LIDAR_ENABLED:-true}" = "true" ] && [ "${LIDAR_CONNECTION:-uart}" = "uart" ] && [ -n "${LIDAR_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$LIDAR_UART_DEVICE")\", SYMLINK+=\"lidar\", MODE=\"0666\""
  fi

  # TF-Luna front
  if effective_tfluna_front_enabled && [ -n "${TFLUNA_FRONT_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$TFLUNA_FRONT_UART_DEVICE")\", SYMLINK+=\"tfluna_front\", MODE=\"0666\""
  fi

  # TF-Luna edge
  if effective_tfluna_edge_enabled && [ -n "${TFLUNA_EDGE_UART_DEVICE:-}" ]; then
    echo "KERNEL==\"$(basename "$TFLUNA_EDGE_UART_DEVICE")\", SYMLINK+=\"tfluna_edge\", MODE=\"0666\""
  fi
}

install_udev_rules() {
  step "Installing udev rules"
  require_root_for "udev rules"

  local rules_file="$UDEV_RULES_FILE"
  local tmpfile
  tmpfile="$(mktemp)"

  if ! {
    build_static_udev_rules
    echo ""
    build_dynamic_udev_rules
  } > "$tmpfile"; then
    rm -f "$tmpfile"
    return 1
  fi

  local changed=false

  if [ ! -f "$rules_file" ]; then
    changed=true
  elif ! cmp -s "$tmpfile" "$rules_file"; then
    changed=true
  fi

  if $changed; then
    $SUDO cp "$tmpfile" "$rules_file"
    $SUDO udevadm control --reload-rules
    $SUDO udevadm trigger
    info "udev rules installed/updated"
  else
    info "udev rules already up to date"
  fi

  rm -f "$tmpfile"

  # Verify symlinks were created — UART devices may not exist until reboot
  local gnss_backend
  local needs_reboot=false

  gnss_backend="$(effective_gnss_backend 2>/dev/null || true)"

  if [ "$gnss_backend" != "disabled" ] && [ "${GPS_CONNECTION:-usb}" = "uart" ] && [ -n "${GPS_UART_DEVICE:-}" ]; then
    if [ ! -e "$GPS_UART_DEVICE" ]; then
      warn "GPS UART device $GPS_UART_DEVICE does not exist yet (UART overlay needs reboot)"
      needs_reboot=true
    elif [ ! -e "${GPS_PORT:-/dev/gps}" ]; then
      warn "GPS symlink ${GPS_PORT:-/dev/gps} not created — check udev rules"
    else
      info "GPS symlink: ${GPS_PORT:-/dev/gps} -> $(readlink -f "${GPS_PORT:-/dev/gps}")"
    fi
  elif [ "$gnss_backend" != "disabled" ] && [ -n "${GPS_BY_ID:-}" ]; then
    if [ ! -e "${GPS_BY_ID}" ]; then
      warn "GPS by-id device ${GPS_BY_ID} does not exist"
    else
      info "GPS device: ${GPS_BY_ID} -> $(readlink -f "${GPS_BY_ID}")"
      if [ ! -e "${GPS_PORT:-/dev/gps}" ]; then
        warn "GPS symlink ${GPS_PORT:-/dev/gps} not created — selected source device is ${GPS_BY_ID}"
      else
        info "GPS symlink: ${GPS_PORT:-/dev/gps} -> $(readlink -f "${GPS_PORT:-/dev/gps}")"
      fi
    fi
  fi

  if [ "${LIDAR_ENABLED:-true}" = "true" ] && [ "${LIDAR_CONNECTION:-}" = "uart" ] && [ -n "${LIDAR_UART_DEVICE:-}" ]; then
    if [ ! -e "$LIDAR_UART_DEVICE" ]; then
      warn "LiDAR UART device $LIDAR_UART_DEVICE does not exist yet (UART overlay needs reboot)"
      needs_reboot=true
    elif [ ! -e "${LIDAR_PORT:-/dev/lidar}" ]; then
      warn "LiDAR symlink ${LIDAR_PORT:-/dev/lidar} not created — check udev rules"
    else
      info "LiDAR symlink: ${LIDAR_PORT:-/dev/lidar} -> $(readlink -f "${LIDAR_PORT:-/dev/lidar}")"
    fi
  fi

  if $needs_reboot; then
    warn "Some UART devices require a reboot to become available"
    add_issue "Reboot required for UART devices. Run: sudo reboot"
  fi
}
