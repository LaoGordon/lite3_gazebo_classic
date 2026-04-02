#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
DEFAULT_WORLD="$SCRIPT_DIR/libraries/quadruped_playground/worlds/test_world.world"
DEFAULT_MAP="$SCRIPT_DIR/navigation/quadruped_nav_bringup/maps/floor_small_test.yaml"
DEFAULT_NAV_RVIZ="$SCRIPT_DIR/navigation/quadruped_nav_bringup/rviz/nav_validation.rviz"

WORLD_FILE="${1:-$DEFAULT_WORLD}"
MAP_YAML="${MAP_YAML:-$DEFAULT_MAP}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-true}"
NAV_RVIZ="${NAV_RVIZ:-true}"
ENABLE_PCD_SAVE="${ENABLE_PCD_SAVE:-false}"
GAZEBO_DELAY="${GAZEBO_DELAY:-8}"
FASTLIVO_DELAY="${FASTLIVO_DELAY:-6}"
BRIDGE_DELAY="${BRIDGE_DELAY:-4}"
STATIC_MAP_DELAY="${STATIC_MAP_DELAY:-3}"
NAV2_DELAY="${NAV2_DELAY:-3}"

if [ ! -f "$WORLD_FILE" ]; then
  echo "world 文件不存在: $WORLD_FILE" >&2
  exit 1
fi

if [ ! -f "$MAP_YAML" ]; then
  echo "地图文件不存在: $MAP_YAML" >&2
  exit 1
fi

PIDS=()

cleanup() {
  local exit_code=$?
  if [ ${#PIDS[@]} -gt 0 ]; then
    echo "[cleanup] stopping launched processes..."
    for pid in "${PIDS[@]}"; do
      kill "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
      kill -9 "$pid" 2>/dev/null || true
    done
  fi
  exit $exit_code
}
trap cleanup INT TERM EXIT

cd "$WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

echo "[1/6] starting Gazebo via run_gazebo_world.sh"
"$SCRIPT_DIR/run_gazebo_world.sh" "$WORLD_FILE" &
PIDS+=("$!")
sleep "$GAZEBO_DELAY"

echo "[2/6] starting FAST-LIVO2"
ros2 launch fast_livo mapping_gazebo.launch.py use_rviz:="$FASTLIO_RVIZ" enable_pcd_save:="$ENABLE_PCD_SAVE" &
PIDS+=("$!")
sleep "$FASTLIVO_DELAY"

echo "[3/6] starting fastlivo_nav_bridge"
ros2 launch fastlivo_nav_bridge bridge.launch.py &
PIDS+=("$!")
sleep "$BRIDGE_DELAY"

echo "[4/6] starting static map server"
ros2 launch quadruped_nav_bringup static_map.launch.py map_yaml:="$MAP_YAML" &
PIDS+=("$!")

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
PIDS+=("$!")
sleep "$STATIC_MAP_DELAY"

echo "[5/6] starting navigation stack"
ros2 launch quadruped_nav_bringup navigation_main.launch.py enable_static_map:=false enable_nav2:=true map_yaml:="$MAP_YAML" &
PIDS+=("$!")
sleep "$NAV2_DELAY"

echo "[6/6] starting navigation RViz"
if [ "$NAV_RVIZ" = "true" ] && [ -f "$DEFAULT_NAV_RVIZ" ]; then
  rviz2 -d "$DEFAULT_NAV_RVIZ" &
  PIDS+=("$!")
fi

echo
echo "Navigation validation stack started."
echo "  world: $WORLD_FILE"
echo "  map:   $MAP_YAML"
echo "  FAST-LIVO RViz: $FASTLIO_RVIZ"
echo "  Nav RViz:      $NAV_RVIZ"
echo
echo "Press Ctrl-C in this terminal to stop the full stack."

wait
