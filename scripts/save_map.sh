#!/usr/bin/env bash
# Save the map from a running mapping mission and install it, together with
# the dock poses captured in the same run.
#
#   ~/modysseus/scripts/save_map.sh [name]     (default name: arena_real)
#
# Run it while the mapping launch is still up: the pose graph only exists
# inside the running slam_toolbox. Safe to run from any directory -- every path
# below is absolute, and the build always lands in this workspace's install/.
#
# Why the build covers three packages, not just localization: the stack reads
# the INSTALLED copies of its config. capture_dock_pose writes dock poses into
# the source files perception/config/docking.yaml and mission/config/docks.yaml,
# and those poses are coordinates on this run's map. Installing the new map
# without them leaves docking steering to the previous map's coordinates, which
# lands the robot at the wrong spot while still reporting success.
set -euo pipefail

name="${1:-arena_real}"
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
maps="$ws/src/localization/maps"
base="$maps/$name"

set +u
source /opt/ros/jazzy/setup.bash
set -u

start=$(date +%s)
# When the previous map was saved. Docks captured before then were measured
# on that older map.
prev_save=$(stat -c %Y "$base.posegraph" 2>/dev/null || echo 0)

echo "== 1/3 pose graph -> $base.{posegraph,data}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$base'}"

echo "== 2/3 image map -> $base.{pgm,yaml}"
ros2 run nav2_map_server map_saver_cli -f "$base"

stale=0
for ext in posegraph data pgm yaml; do
    f="$base.$ext"
    if [ ! -f "$f" ] || [ "$(stat -c %Y "$f")" -lt "$start" ]; then
        echo "NOT SAVED: $f" >&2
        stale=1
    fi
done
[ "$stale" -eq 0 ] || { echo "Map save incomplete; nothing installed. Is the mapping launch still running?" >&2; exit 1; }
ls -l "$base".*

echo "== 3/3 install map and dock poses into $ws/install"
cd "$ws"
colcon build --packages-select localization perception mission

if [ "$(stat -c %Y "$ws/src/perception/config/docking.yaml")" -le "$prev_save" ]; then
    echo "WARNING: no dock was captured since the previous map was saved, so the" \
         "installed dock poses belong to that older map and docking will go to the" \
         "wrong place. Map again and capture the docks (capture_dock_pose) in the same run." >&2
fi

echo
echo "Done. In every terminal that will run the stack:"
echo "  source $ws/install/setup.bash"
