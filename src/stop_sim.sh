#!/bin/bash

# Stop whatever an earlier scenario left running in this container.
# Every `make <scenario>` runs this first; `make stop` runs it alone.
#
# Closing the terminal of a `make <scenario>` does not stop it (docker exec
# leaves the processes running), and gzserver can outlive Ctrl+C. The
# leftovers break the next run:
#   - the old gzserver holds Gazebo's master port, so the new one exits and
#     Gazebo shows the old world, with the robot and person wherever they were;
#   - old nodes share names with the new ones: Nav2's lifecycle manager reaches
#     the old map_server/amcl, bringup aborts, and RViz reports
#     "Frame [map] does not exist";
#   - two security_guard_bt publish /cmd_vel, two person_controllers steer
#     the person.
#
# Uses /proc only: the image does not guarantee procps (pgrep, ps).

PATTERN='ros2 launch|gzserver|gzclient|rviz2|component_container|/opt/ros/humble/lib/nav2_|robot_state_publisher|spawn_entity|/install/my_bot/lib/my_bot/'

parent_of() {
    local stat
    stat=$(cat "/proc/$1/stat" 2>/dev/null) || return 1
    set -- ${stat##*) }  # fields after "(comm) ": state ppid ...
    echo "$2"
}

# This script and the shells that started it: the scenario's own
# `bash -c "... ros2 launch ..."` matches the pattern.
skip=" $$ "
pid=$$
while pid=$(parent_of "$pid") && [ "$pid" -gt 1 ]; do
    skip+="$pid "
done

victims=()
for dir in /proc/[0-9]*; do
    pid=${dir#/proc/}
    [[ $skip == *" $pid "* ]] && continue
    cmd=$(tr '\0' ' ' < "$dir/cmdline" 2>/dev/null) || continue
    [[ -n $cmd && $cmd =~ $PATTERN ]] || continue
    victims+=("$pid")
    [ ${#victims[@]} -eq 1 ] && echo 'Stopping processes left over from an earlier run:'
    echo "  $pid ${cmd:0:100}"
done
[ ${#victims[@]} -eq 0 ] && exit 0

kill -INT "${victims[@]}" 2>/dev/null
for _ in $(seq 50); do  # up to 5 s to shut down cleanly
    alive=()
    for pid in "${victims[@]}"; do
        kill -0 "$pid" 2>/dev/null && alive+=("$pid")
    done
    [ ${#alive[@]} -eq 0 ] && exit 0
    sleep 0.1
done
echo "  ${#alive[@]} did not stop on Ctrl+C, killing them"
kill -KILL "${alive[@]}" 2>/dev/null
exit 0
