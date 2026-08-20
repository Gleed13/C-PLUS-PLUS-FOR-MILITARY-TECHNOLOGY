#!/usr/bin/env bash
# Запуск одного сценарію із записом bag-файлу.
# Використання: ./run_scenario.sh <scenario_name> [run_seconds]
set -e

scenario="$1"
run_seconds="${2:-40}"
root="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$root/robot_ws/install/setup.bash"

rm -rf "$root/bags/$scenario"
mkdir -p "$root/bags"

# Рекордер стартує першим, щоб не втратити перші повідомлення.
ros2 bag record -a -o "$root/bags/$scenario" > "/tmp/${scenario}_bag.log" 2>&1 &
bag_pid=$!
sleep 4

timeout "$run_seconds" ros2 launch underground_world system.launch.py \
  "scenario:=${scenario}.yaml" > "/tmp/${scenario}_run.log" 2>&1 &
launch_pid=$!

sleep $((run_seconds - 15))
echo "--- ${scenario} result ---"
timeout 10 ros2 topic echo /robot/result --once
echo "--- ${scenario} metrics ---"
timeout 10 ros2 topic echo /robot/metrics --once

kill -INT "$launch_pid" 2>/dev/null || true
sleep 3
kill -TERM "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true

ros2 bag info "$root/bags/$scenario"
