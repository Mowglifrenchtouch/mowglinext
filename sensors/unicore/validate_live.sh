#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MOWGLI_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

candidate_paths=()
if [ -n "${UM982_DRIVER_DIR:-}" ]; then
  candidate_paths+=("${UM982_DRIVER_DIR}")
fi
candidate_paths+=("${MOWGLI_ROOT}/../UM982Driver")

for candidate in "${candidate_paths[@]}"; do
  [ -n "$candidate" ] || continue
  tool_path="${candidate}/tools/um982_live_validate.py"
  if [ -f "$tool_path" ]; then
    exec python3 "$tool_path" "$@"
  fi
done

default_driver_dir="${MOWGLI_ROOT}/../UM982Driver"

cat >&2 <<EOF
[validate_live.sh] ERROR: UM982Driver live validator not found.
[validate_live.sh] Looked for:
[validate_live.sh]   - ${UM982_DRIVER_DIR:-<unset>}
[validate_live.sh]   - ${default_driver_dir}
[validate_live.sh] Recommended:
[validate_live.sh]   export UM982_DRIVER_DIR=/path/to/UM982Driver
[validate_live.sh]   python3 /path/to/UM982Driver/tools/um982_live_validate.py --help
EOF

exit 1
