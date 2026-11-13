#!/bin/bash

# Headless smoke test of the whole security-guard stack (Phase 5b).
# `make smoke` runs it in the container; the CI smoke workflow
# (.github/workflows/smoke.yml) runs it on a GitHub runner.
#
# One scripted scenario (default: smoke, from config/scenarios.yaml) with
# no Gazebo window, no RViz and no rosbag; then the report must show that
# every part came up and the loop closed (scenario_metrics.smoke_failures):
# the scenario finished, the person was detected, the guard raised its
# alert. The run's own targets (detection time, tracking) are not checked
# here: CI reports them in the scenario summary table instead (Phase 7).
#
#   src/smoke_test.sh [scenario]
#
# Environment (all optional):
#   SMOKE_OUT           report + launch.log directory
#                       (default ~/.ros/autonav_scenarios/smoke_<date-time>)
#   SMOKE_SIM_TIMEOUT   evaluator timeout in sim seconds (default 300)
#   SMOKE_WALL_TIMEOUT  wall-clock limit in seconds (default 1800); then the
#                       launch gets Ctrl+C and the evaluator scores what it has
#   SMOKE_REPLAY        true: also record a bag with the tracker's camera view
#                       and export it as a web demo replay into $out/replay
#                       (my_bot.replay_export; the bag is left in $out/bag)
#
# Exit status: 0 = pass, 1 = fail. Needs the workspace sourced.

set -u

scenario=${1:-smoke}
out=${SMOKE_OUT:-$HOME/.ros/autonav_scenarios/smoke_$(date +%Y%m%d-%H%M%S)}
mkdir -p "$out"

# timeout signals its own child, so it wraps ros2 launch directly
record=(record:=false)
if [ "${SMOKE_REPLAY:-false}" = true ]; then
    record=(record:=true record_images:=true)
fi
launch=(timeout --signal=INT --kill-after=60 "${SMOKE_WALL_TIMEOUT:-1800}"
        ros2 launch my_bot scenario.launch.py "scenario:=$scenario"
        "out_dir:=$out" "${record[@]}" "timeout_secs:=${SMOKE_SIM_TIMEOUT:-300}")
# gzserver renders the camera with OpenGL even without a window: with no X
# display (CI) it needs a virtual one.
if [ -z "${DISPLAY:-}" ] && command -v xvfb-run > /dev/null; then
    launch=(xvfb-run -a -s "-screen 0 1280x1024x24" "${launch[@]}")
fi

echo "smoke test: scenario $scenario, output in $out"
"${launch[@]}" > "$out/launch.log" 2>&1
echo "launch exited with status $? (log: $out/launch.log)"

# gzserver can outlive the launch (see stop_sim.sh)
bash "$(dirname "$0")/stop_sim.sh" > /dev/null 2>&1

[ -f "$out/report.json" ] && cat "$out/report.json"
if [ "${SMOKE_REPLAY:-false}" = true ]; then
    # A replay is a by-product: its failure does not fail the smoke test
    python3 -m my_bot.replay_export "$out" "$out/replay" \
        || echo "replay export failed (see above)"
fi
python3 -m my_bot.scenario_metrics smoke "$out/report.json"
status=$?

# On failure, put the telling part of launch.log in the job's own log: CI
# keeps launch.log only as an artifact, and a startup failure (Nav2 bringup
# aborted, a node died, Gazebo stalled) is invisible in the report.
if [ "$status" -ne 0 ] && [ -f "$out/launch.log" ]; then
    echo "--- launch.log: errors, Nav2 lifecycle, guard (first 150) ---"
    grep -aE '\[(ERROR|FATAL)\]|CRITICAL|Failed to|heartbeat|Traceback|Error:|process has died|lifecycle_manager|security_guard_bt' \
        "$out/launch.log" | head -n 150
    echo "--- launch.log: last 60 lines ---"
    tail -n 60 "$out/launch.log"
fi
exit "$status"
