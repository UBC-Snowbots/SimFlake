#!/usr/bin/env bash
# Drivetrain test bench, headless: builds a player with Assets/Scenes/Tests/DriveTest.unity and runs it.
# No ROS needed. The Unity editor must be closed (project lock).
#
#   tools/drive_bench.sh                         # all rovers, all grounds, all tests
#   tools/drive_bench.sh --setup                 # first re-apply DrivetrainSetup (specs -> prefabs, physics settings, scene)
#   tools/drive_bench.sh --no-build -- --drive-test-rovers mini_rover --drive-test-only spin,slope --drive-test-trace
#
# Anything after `--` goes to the player (see DriveTestRunner.cs for the flags). Results: table on stdout,
# JSON in $OUT/results.json, full log in $OUT/player.log.
set -euo pipefail
cd "$(dirname "$0")/.."
U=${UNITY:-$HOME/Unity/Hub/Editor/$(sed -n 's/m_EditorVersion: //p' rover_sim/ProjectSettings/ProjectVersion.txt)/Editor/Unity}
OUT=${OUT:-/tmp/simflake_drive_bench}
mkdir -p "$OUT"
build=1 setup=0
while [ $# -gt 0 ]; do
  case "$1" in
    --setup) setup=1 ;;
    --no-build) build=0 ;;
    --) shift; break ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done

if [ $setup = 1 ]; then
  echo "== DrivetrainSetup"
  "$U" -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.DrivetrainSetup.SetUpFromCommandLine -logFile "$OUT/setup.log" \
    || { grep -E "error|Exception" "$OUT/setup.log" | head; exit 1; }
  grep "\[Drivetrain\]" "$OUT/setup.log" || true
fi
if [ $build = 1 ]; then
  echo "== building player"
  "$U" -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.SimFlakeTools.BuildLinux \
    -scenes Assets/Scenes/Tests/DriveTest.unity -out "$OUT/build/rover_sim.x86_64" -logFile "$OUT/build.log" \
    || { grep -E "error CS|Build " "$OUT/build.log" | head; exit 1; }
fi
echo "== running bench"
"$OUT/build/rover_sim.x86_64" -batchmode -nographics --drive-test --drive-test-out "$OUT/results.json" \
  --rosbridge-url ws://127.0.0.1:9 -logFile "$OUT/player.log" "$@"
sed -n '/\[DriveTest\] RESULTS/,/^$/p' "$OUT/player.log"
grep "\[DriveTest\] done" "$OUT/player.log" || { echo "bench did not finish, see $OUT/player.log"; exit 1; }
