#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/lca_scenes.compose.yml"
COMPOSE_PROJECT_BASE="lca_scenes"
LCA_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
HOST_FASTEX_WS_DEFAULT="$(cd "${LCA_ROOT}/../.." && pwd)"

: "${HOST_FASTEX_WS:=${HOST_FASTEX_WS_DEFAULT}}"
: "${LOCAL_UID:=$(id -u)}"
: "${LOCAL_GID:=$(id -g)}"
: "${DOCKER_XAUTH:=/tmp/.docker.xauth}"
: "${IS_LAUNCH_RVIZ:=true}"
: "${IS_LOGGING:=true}"
: "${IS_LAUNCH_DATA_VIS:=${IS_LOGGING}}"
: "${STACK_SUFFIX:=}"

if [ -n "${COMPOSE_PROJECT_NAME:-}" ]; then
  COMPOSE_PROJECT="${COMPOSE_PROJECT_NAME}"
elif [ -n "${STACK_SUFFIX}" ]; then
  COMPOSE_PROJECT="${COMPOSE_PROJECT_BASE}_${STACK_SUFFIX}"
else
  COMPOSE_PROJECT="${COMPOSE_PROJECT_BASE}"
fi

export HOST_FASTEX_WS
export LOCAL_UID
export LOCAL_GID
export DOCKER_XAUTH
export IS_LOGGING
export IS_LAUNCH_DATA_VIS
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT}"

ALL_SCENES=(cave forest indoor)
declare -A SCENE_SERVICE=(
  [cave]=lca_cave
  [forest]=lca_forest
  [indoor]=lca_indoor
)

usage() {
  cat <<'USAGE'
Usage:
  bash docker/run_lca_scenes.sh start [scene ...]
  bash docker/run_lca_scenes.sh stop [scene ...]
  bash docker/run_lca_scenes.sh logs [scene ...]
  bash docker/run_lca_scenes.sh ps
  bash docker/run_lca_scenes.sh env
  bash docker/run_lca_scenes.sh down

Examples:
  bash docker/run_lca_scenes.sh start cave
  bash docker/run_lca_scenes.sh start forest indoor
  IS_LAUNCH_RVIZ=true bash docker/run_lca_scenes.sh start cave
  LCA_IMAGE=ros:fastex_experiment bash docker/run_lca_scenes.sh start forest
  STACK_SUFFIX=test bash docker/run_lca_scenes.sh start indoor

Env overrides:
  HOST_FASTEX_WS=/abs/path/to/FASTEX_ws
  LCA_IMAGE=ros:fastex_experiment
  DOCKER_XAUTH=/tmp/.docker.xauth
  IS_LAUNCH_RVIZ=true
  IS_LOGGING=true
  IS_LAUNCH_DATA_VIS=true
  STACK_SUFFIX=test
  COMPOSE_PROJECT_NAME=my_lca_scenes

Scenes: cave forest indoor
If no scene is given, all scenes are selected.
USAGE
}

compose_cmd() {
  docker compose -p "${COMPOSE_PROJECT}" -f "${COMPOSE_FILE}" "$@"
}

prepare_x11_auth() {
  mkdir -p "$(dirname "${DOCKER_XAUTH}")"

  if [ -n "${DISPLAY:-}" ] && command -v xauth >/dev/null 2>&1; then
    local xauth_list
    xauth_list="$(xauth nlist "${DISPLAY}" 2>/dev/null | tail -n 1 | sed -e 's/^..../ffff/' || true)"
    if [ -n "${xauth_list}" ]; then
      : > "${DOCKER_XAUTH}"
      echo "${xauth_list}" | xauth -f "${DOCKER_XAUTH}" nmerge - >/dev/null 2>&1 || true
    else
      touch "${DOCKER_XAUTH}"
    fi
  else
    touch "${DOCKER_XAUTH}"
    if [ -z "${DISPLAY:-}" ]; then
      echo "[lca-scenes-docker] DISPLAY is empty; GUI nodes will not be able to open windows." >&2
    elif ! command -v xauth >/dev/null 2>&1; then
      echo "[lca-scenes-docker] xauth not found; created empty ${DOCKER_XAUTH}." >&2
    fi
  fi

  chmod a+r "${DOCKER_XAUTH}" 2>/dev/null || true
}

resolve_services() {
  local scenes=("$@")
  local services=()

  if [ "${#scenes[@]}" -eq 0 ]; then
    scenes=("${ALL_SCENES[@]}")
  fi

  local scene
  for scene in "${scenes[@]}"; do
    if [[ -z "${SCENE_SERVICE[${scene}]:-}" ]]; then
      echo "[lca-scenes-docker] Unsupported scene: ${scene}" >&2
      echo "Supported scenes: ${ALL_SCENES[*]}" >&2
      exit 1
    fi
    services+=("${SCENE_SERVICE[${scene}]}")
  done

  printf '%s\n' "${services[@]}"
}

if [ ! -f "${COMPOSE_FILE}" ]; then
  echo "[lca-scenes-docker] Compose file not found: ${COMPOSE_FILE}" >&2
  exit 1
fi

if [[ "${HOST_FASTEX_WS}" != /* ]]; then
  echo "[lca-scenes-docker] HOST_FASTEX_WS must be an absolute path: ${HOST_FASTEX_WS}" >&2
  exit 1
fi

if [ ! -d "${HOST_FASTEX_WS}" ]; then
  echo "[lca-scenes-docker] HOST_FASTEX_WS does not exist: ${HOST_FASTEX_WS}" >&2
  exit 1
fi

if [ ! -d "${HOST_FASTEX_WS}/src/LCAExplorer" ]; then
  echo "[lca-scenes-docker] Missing directory: ${HOST_FASTEX_WS}/src/LCAExplorer" >&2
  exit 1
fi

ACTION="${1:-start}"
if [ "$#" -gt 0 ]; then
  shift
fi

case "${ACTION}" in
  start)
    mapfile -t SERVICES < <(resolve_services "$@")
    prepare_x11_auth
    echo "[lca-scenes-docker] COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT}"
    echo "[lca-scenes-docker] HOST_FASTEX_WS=${HOST_FASTEX_WS}"
    echo "[lca-scenes-docker] LOCAL_UID=${LOCAL_UID}, LOCAL_GID=${LOCAL_GID}"
    echo "[lca-scenes-docker] DISPLAY=${DISPLAY:-<empty>}"
    echo "[lca-scenes-docker] DOCKER_XAUTH=${DOCKER_XAUTH}"
    compose_cmd up -d "${SERVICES[@]}"
    compose_cmd ps
    ;;
  stop)
    mapfile -t SERVICES < <(resolve_services "$@")
    compose_cmd stop "${SERVICES[@]}"
    ;;
  logs)
    mapfile -t SERVICES < <(resolve_services "$@")
    compose_cmd logs -f "${SERVICES[@]}"
    ;;
  ps)
    compose_cmd ps
    ;;
  env)
    echo "COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT}"
    echo "HOST_FASTEX_WS=${HOST_FASTEX_WS}"
    echo "LOCAL_UID=${LOCAL_UID}"
    echo "LOCAL_GID=${LOCAL_GID}"
    echo "DISPLAY=${DISPLAY:-}"
    echo "DOCKER_XAUTH=${DOCKER_XAUTH}"
    echo "IS_LAUNCH_RVIZ=${IS_LAUNCH_RVIZ:-false}"
    echo "IS_LOGGING=${IS_LOGGING}"
    echo "IS_LAUNCH_DATA_VIS=${IS_LAUNCH_DATA_VIS}"
    ;;
  down)
    compose_cmd down --remove-orphans
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "[lca-scenes-docker] Unknown action: ${ACTION}" >&2
    usage
    exit 1
    ;;
esac
