#!/usr/bin/env bash
# Arm tests, headless (no ROS): stepper hold/move/stall, gripper squeeze, breakage + repair, arm on chassis_2024.
# Builds a player with ArmScene and runs it with --arm-test. The Unity editor must be closed (project lock).
#
#   tools/arm_test.sh                          # build + all tests (~3 min)
#   tools/arm_test.sh --no-build -- --arm-test-only grip,rover
set -euo pipefail
cd "$(dirname "$0")/.."
U=${UNITY:-$HOME/Unity/Hub/Editor/$(sed -n 's/m_EditorVersion: //p' rover_sim/ProjectSettings/ProjectVersion.txt)/Editor/Unity}
OUT=${OUT:-/tmp/simflake_arm_test}
mkdir -p "$OUT"
build=1
while [ $# -gt 0 ]; do
  case "$1" in
    --no-build) build=0 ;;
    --) shift; break ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done
if [ $build = 1 ]; then
  echo "== building player"
  "$U" -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.SimFlakeTools.BuildLinux \
    -scenes Assets/Scenes/ArmScene.unity -out "$OUT/build/rover_sim.x86_64" -logFile "$OUT/build.log" \
    || { grep -E "error CS|Build " "$OUT/build.log" | head; exit 1; }
fi
echo "== running arm tests"
status=0
"$OUT/build/rover_sim.x86_64" -batchmode -nographics --arm-test --rosbridge-url ws://127.0.0.1:9 -logFile "$OUT/player.log" "$@" || status=$?
grep "\[ArmTest\]" "$OUT/player.log" | sed 's/^\[ArmTest\] //'
grep -q "\[ArmTest\] done" "$OUT/player.log" || { echo "arm test did not finish, see $OUT/player.log"; exit 1; }
exit $status
