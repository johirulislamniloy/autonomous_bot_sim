#!/usr/bin/env bash
# Build and launch the competition simulation, then press START automatically.
#
#   ./run_sim.sh                 # obstacle round + GUI + RViz + auto start
#   ./run_sim.sh mode:=open      # open round (3x3 m ring, random start)
#   ./run_sim.sh gui:=false rviz:=false seed:=123
#   ./run_sim.sh auto_start:=false   # start manually with the start_button
#   ./run_sim.sh randomize:=false start_paused:=true auto_start:=false
#                                # paused, place obstacles yourself
#   ./run_sim.sh randomize:=false manual_layout:=$PWD/config/manual_example.yaml
#   ./run_sim.sh mode:=astar     # obstacle round: map-aware A* (default)
#   ./run_sim.sh mode:=scan      # obstacle round: reactive scan-and-go
#   ./run_sim.sh mode:=bug2      # obstacle round: reactive Bug2
#   ./run_sim.sh mode:=open open_clockwise:=true
#                                # open round on real hardware (fixed direction)
#
# Any extra arguments are passed straight to `ros2 launch`.
set -eo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
PKG="autonomous_bot_sim"

cd "$WS"

# Load ROS if this shell has not sourced it already.
if [ -z "${ROS_DISTRO:-}" ]; then
    # shellcheck disable=SC1090
    source /opt/ros/*/setup.bash
fi

colcon build --packages-select "$PKG" --symlink-install
# shellcheck disable=SC1091
source install/setup.bash

# Auto-start unless the caller explicitly overrides it.
ARGS=("$@")
if [[ " $* " != *" auto_start:="* ]]; then
    ARGS+=("auto_start:=true")
fi

exec ros2 launch "$PKG" sim.launch.py "${ARGS[@]}"
