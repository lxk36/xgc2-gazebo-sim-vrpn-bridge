#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE="ros-noetic-xgc2-gazebo-sim-vrpn-bridge"
ROS_PACKAGE="gazebo_sim_vrpn_bridge"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
PKG_ROOT="$(mktemp -d)"

cleanup() {
  rm -rf "${PKG_ROOT}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}" "${PKG_ROOT}/DEBIAN" "${PKG_ROOT}/usr/share/doc/${PACKAGE}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  if [[ -e "${src}" ]]; then
    mkdir -p "${PKG_ROOT}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${PKG_ROOT}${src#${INSTALL_ROOT}}"
  fi
}

copy_path "${PREFIX_ROOT}/share/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/lib/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/include/${ROS_PACKAGE}"
install -D -m 0644 \
  "${REPO_ROOT}/process-definitions/xgc2-gazebo-sim-vrpn-bridge.json" \
  "${PKG_ROOT}/usr/share/xgc2/process-definitions/xgc2-gazebo-sim-vrpn-bridge.json"

cat > "${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: libxgc2-math-dev (>= 0.5.6-6~focal), ros-noetic-gazebo-msgs, ros-noetic-geometry-msgs, ros-noetic-roscpp, ros-noetic-tf2, ros-noetic-tf2-ros, ros-noetic-vrpn, ros-noetic-vrpn-client-ros
Description: XGC2 Gazebo Classic model pose to VRPN tracker bridge
EOF

printf '%s package\n' "${PACKAGE}" > "${PKG_ROOT}/usr/share/doc/${PACKAGE}/README"
find "${PKG_ROOT}" -type d -exec chmod 0755 {} +
find "${PKG_ROOT}" -type f -exec chmod 0644 {} +
if [[ -d "${PKG_ROOT}${PREFIX}/lib/${ROS_PACKAGE}" ]]; then
  find "${PKG_ROOT}${PREFIX}/lib/${ROS_PACKAGE}" -type f -exec chmod 0755 {} +
fi
chmod 0755 "${PKG_ROOT}/DEBIAN"

fakeroot dpkg-deb --build "${PKG_ROOT}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
