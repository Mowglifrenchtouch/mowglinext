#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

failures=0

assert_contains() {
  local label="${1:?assert_contains: missing label}"
  local needle="${2:?assert_contains: missing needle}"
  local haystack="${3-}"

  if [[ "$haystack" != *"$needle"* ]]; then
    echo "FAIL: $label"
    echo "  missing: $needle"
    echo "  actual:  $haystack"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

make_fake_ros2() {
  mkdir -p "$WORKDIR/bin"
  cat >"$WORKDIR/bin/ros2" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >>"$ROS2_LOG_PATH"
exit 0
EOF
  chmod +x "$WORKDIR/bin/ros2"
}

make_config() {
  cat >"$WORKDIR/mowgli_robot.yaml" <<'EOF'
gps_port: /dev/gps
gps_baudrate: 921600
unicore_auto_configure: false
ntrip_enabled: false
EOF
}

run_profile() {
  local profile="${1:?run_profile: missing profile}"
  local log_file="$WORKDIR/${profile}.log"
  PATH="$WORKDIR/bin:$PATH" \
  ROS2_LOG_PATH="$log_file" \
  MOWGLI_CONFIG_PATH="$WORKDIR/mowgli_robot.yaml" \
  UNICORE_PROFILE="$profile" \
  "$SCRIPT_DIR/start_gps.sh" >/dev/null 2>&1 || true
  cat "$log_file"
}

make_fake_ros2
make_config

NORMAL_ARGS="$(run_profile normal)"
assert_contains "normal disables satellite diagnostics" "enable_satellite_status:=false" "$NORMAL_ARGS"
assert_contains "normal disables RF diagnostics" "enable_rf_status:=false" "$NORMAL_ARGS"
assert_contains "normal disables hardware diagnostics" "enable_hw_status:=false" "$NORMAL_ARGS"
assert_contains "normal disables jamming diagnostics" "enable_jamming_status:=false" "$NORMAL_ARGS"

DEBUG_ARGS="$(run_profile debug)"
assert_contains "debug enables satellite diagnostics" "enable_satellite_status:=true" "$DEBUG_ARGS"
assert_contains "debug enables RF diagnostics" "enable_rf_status:=true" "$DEBUG_ARGS"
assert_contains "debug enables hardware diagnostics" "enable_hw_status:=true" "$DEBUG_ARGS"
assert_contains "debug enables jamming diagnostics" "enable_jamming_status:=true" "$DEBUG_ARGS"

if [ "$failures" -ne 0 ]; then
  echo ""
  echo "$failures test(s) failed."
  exit 1
fi

echo ""
echo "All tests passed."
