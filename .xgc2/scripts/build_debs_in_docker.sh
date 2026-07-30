#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends ca-certificates
    echo "deb [trusted=yes arch=$(dpkg --print-architecture)] https://xgc2.apt.xiaokang.ink focal main" \
      > /etc/apt/sources.list.d/xgc2.list

      if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
        sed "s#${XGC2_APT_BASE_URL:-https://xgc2.apt.xiaokang.ink}#${XGC2_APT_OVERLAY_URL%/}#g" \
          /etc/apt/sources.list.d/xgc2.list \
          > /etc/apt/sources.list.d/00-xgc2-release-train.list
      fi
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      gazebo11 \
      git \
      libgazebo11-dev \
      libxgc2-math-dev \
      netbase \
      python3-nose \
      rsync \
      ros-noetic-gazebo-msgs \
      ros-noetic-gazebo-ros \
      ros-noetic-geometry-msgs \
      ros-noetic-roscpp \
      ros-noetic-roslaunch \
      ros-noetic-rospack \
      ros-noetic-rostest \
      ros-noetic-rosunit \
      ros-noetic-rospy \
      ros-noetic-tf2 \
      ros-noetic-tf2-ros \
      ros-noetic-vrpn \
      ros-noetic-vrpn-client-ros
    dpkg --compare-versions "$(dpkg-query -W -f="\${Version}" libxgc2-math-dev)" ge '0.5.6-6~focal'

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/xgc2-gazebo-sim-vrpn-bridge
    rsync -a --delete /workspace/repo/ /workspace/work/src/xgc2-gazebo-sim-vrpn-bridge/

    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    catkin_make run_tests_gazebo_sim_vrpn_bridge
    catkin_test_results

    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCATKIN_ENABLE_TESTING=OFF

    /workspace/repo/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/ros-noetic-xgc2-gazebo-sim-vrpn-bridge_*.deb
      dpkg-deb -f /workspace/out/ros-noetic-xgc2-gazebo-sim-vrpn-bridge_*.deb Depends \
        | grep -F "libxgc2-math-dev (>= 0.5.6-6~focal)" >/dev/null
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
