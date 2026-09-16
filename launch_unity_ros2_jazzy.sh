#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITY_EDITOR="${UNITY_EDITOR:-$HOME/Unity/Hub/Editor/2022.3.62f3/Editor/Unity}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
UNITY_OPEN_FILE_LIMIT="${UNITY_OPEN_FILE_LIMIT:-65536}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Jazzy setup file was not found: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -x "$UNITY_EDITOR" ]]; then
  echo "Unity Editor was not found or is not executable: $UNITY_EDITOR" >&2
  exit 1
fi

if [[ ! "$UNITY_OPEN_FILE_LIMIT" =~ ^[0-9]+$ ]]; then
  echo "UNITY_OPEN_FILE_LIMIT must be a positive integer" >&2
  exit 1
fi

hard_open_file_limit="$(ulimit -Hn)"
target_open_file_limit="$UNITY_OPEN_FILE_LIMIT"
if [[ "$hard_open_file_limit" != "unlimited" ]] &&
   ((target_open_file_limit > hard_open_file_limit)); then
  target_open_file_limit="$hard_open_file_limit"
fi
ulimit -Sn "$target_open_file_limit"

set +u
source "$ROS_SETUP"
set -u

echo "Launching Unity with ROS_DISTRO=$ROS_DISTRO"
echo "Using Unity Editor: $UNITY_EDITOR"
echo "Project path: $PROJECT_DIR"
echo "Open file limit: $(ulimit -Sn)"

exec "$UNITY_EDITOR" -projectPath "$PROJECT_DIR" "$@"
