#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PRODUCT_PATH=".xgc2/product.yml"
MODE="worktree"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ci)
      MODE="ci"
      shift
      ;;
    --staged)
      MODE="staged"
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

read_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "$1"
}

version_from_git() {
  local ref="$1"
  local tmp
  tmp="$(mktemp)"
  if git -C "${REPO_ROOT}" show "${ref}:${PRODUCT_PATH}" >"${tmp}" 2>/dev/null; then
    read_version "${tmp}"
  fi
  rm -f "${tmp}"
}

version_from_index() {
  local tmp
  tmp="$(mktemp)"
  if git -C "${REPO_ROOT}" show ":${PRODUCT_PATH}" >"${tmp}" 2>/dev/null; then
    read_version "${tmp}"
  fi
  rm -f "${tmp}"
}

case "${MODE}" in
  ci)
    current_version="$(version_from_git HEAD)"
    if git -C "${REPO_ROOT}" rev-parse --verify HEAD^ >/dev/null 2>&1; then
      previous_version="$(version_from_git HEAD^)"
    else
      previous_version=""
    fi
    ;;
  staged)
    current_version="$(version_from_index)"
    previous_version="$(version_from_git HEAD)"
    ;;
  worktree)
    current_version="$(read_version "${REPO_ROOT}/${PRODUCT_PATH}")"
    previous_version="$(version_from_git HEAD)"
    ;;
esac

if [[ -z "${current_version}" ]]; then
  echo "${PRODUCT_PATH} must define a top-level version" >&2
  exit 1
fi

if ! dpkg --compare-versions "${current_version}" ge "${current_version}"; then
  echo "invalid Debian package version: ${current_version}" >&2
  exit 1
fi

if [[ -z "${previous_version}" ]]; then
  echo "Previous product version missing; version ${current_version} accepted"
  exit 0
fi

if ! dpkg --compare-versions "${current_version}" gt "${previous_version}"; then
  echo "product version must increase on every product repository update" >&2
  echo "previous: ${previous_version}" >&2
  echo "current:  ${current_version}" >&2
  exit 1
fi

echo "Product version increased: ${previous_version} -> ${current_version}"
