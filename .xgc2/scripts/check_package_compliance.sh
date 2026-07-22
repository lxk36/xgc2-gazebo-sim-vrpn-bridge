#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

bash -n .xgc2/scripts/*.sh

if git ls-files | grep -E '(^|/)(build|devel|install|install-root|\.work|\.ci|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|install-root|\.work|\.ci|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .github/workflows/ci.yml
  .github/workflows/release.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/package_debs.sh
  CMakeLists.txt
  README.md
  package.xml
  process-definitions/xgc2-gazebo-sim-vrpn-bridge.json
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

grep -q "id: xgc2-gazebo-sim-vrpn-bridge" .xgc2/product.yml
grep -Eq '^version: [0-9]+\.[0-9]+\.[0-9]+-[0-9]+$' .xgc2/product.yml
grep -q "ros-noetic-xgc2-gazebo-sim-vrpn-bridge" .xgc2/product.yml
grep -q "PACKAGE=\"ros-noetic-xgc2-gazebo-sim-vrpn-bridge\"" .xgc2/scripts/package_debs.sh
grep -q "libxgc2-math-dev (>= 0.5.6-6~focal)" .xgc2/scripts/package_debs.sh
grep -q "gazebo_sim_vrpn_bridge" package.xml
grep -q "gazebo_vrpn_server_node" CMakeLists.txt
grep -q "vrpn_server.launch" .xgc2/scripts/check_installed_packages.sh
python3 - <<'PY'
import json
manifest = json.load(open('process-definitions/xgc2-gazebo-sim-vrpn-bridge.json'))
assert [item['id'] for item in manifest['definitions']] == ['gazebo-vrpn-server']
PY

echo "Package compliance checks passed."
