#!/usr/bin/env bash
# =============================================================================
# Runtime wiring for GNSS_BACKEND=ublox
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"

compose_file="$REPO_ROOT/install/compose/docker-compose.gps.yml"
launch_file="$REPO_ROOT/ros2/src/mowgli_bringup/launch/ublox_gnss.launch.py"
config_file="$REPO_ROOT/ros2/src/mowgli_bringup/config/ublox_gnss.yaml"

section "Compose wiring"

compose_content="$(cat "$compose_file")"
assert_not_contains "compose does not pass legacy ublox_baud_rate override" 'ublox_baud_rate:=' "$compose_content"
assert_contains "compose passes canonical GPS device path" 'GPS_DEVICE_PATH: ${GPS_PORT:-${UBLOX_DEVICE_SERIAL_STRING:-/dev/gps}}' "$compose_content"
assert_contains "compose passes GPS_BAUD through shared gps fragment" 'GPS_BAUD: ${GPS_BAUD:-921600}' "$compose_content"
assert_contains "compose documents legacy ublox fallback path" 'fall back to UBLOX_DEVICE_SERIAL_STRING' "$compose_content"

section "Launch wiring"

launch_content="$(cat "$launch_file")"
assert_contains "launch still injects device serial string" '"DEVICE_SERIAL_STRING": device_serial_string' "$launch_content"
assert_not_contains "launch no longer declares ublox_baud_rate" '"ublox_baud_rate"' "$launch_content"
assert_not_contains "launch no longer injects BAUD_RATE" '"BAUD_RATE":' "$launch_content"
assert_contains "launch remaps hp fix output into /gps/fix" 'remappings=[("/fix", "/gps/fix")]' "$launch_content"
assert_contains "launch uses shared navsat adapter executable" 'executable="navsat_to_absolute_pose_node"' "$launch_content"

section "Config presence"

assert_file_exists "ublox runtime config file exists" "$config_file"
config_content="$(cat "$config_file")"
assert_contains "ublox runtime config declares ublox_dgnss section" 'ublox_dgnss:' "$config_content"
assert_contains "ublox runtime config declares adapter section" 'ublox_gnss_adapter:' "$config_content"
assert_contains "ublox runtime config documents dedicated driver" 'GNSS_BACKEND=ublox uses the dedicated USB/libusb u-blox driver.' "$config_content"
assert_contains "ublox runtime config documents serial ubx fallback" 'Serial UBX receivers should use GNSS_BACKEND=gps with GPS_PROTOCOL=UBX.' "$config_content"

test_summary
