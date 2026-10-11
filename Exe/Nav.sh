#!/bin/bash
# Nav.sh - launches Nav2 for OUR robot, not turtlebot3_navigation2.
#
# Why not turtlebot3_navigation2: it bakes in TurtleBot3 Burger's own
# nav2_params.yaml, which assumes a ~10.5cm-radius circular robot topping
# out at 0.22 m/s. Our robot is a ~43x43cm square-ish chassis with wheels
# sticking out to +/-24cm, so using Burger's footprint would make the
# costmap think we're much smaller than we really are - a real collision
# risk, not just a cosmetic mismatch.
#
# This calls nav2_bringup's OWN bringup_launch.py directly instead, with
# our custom nav2_params.yaml (correct footprint + conservative speed
# limits) sitting alongside this script.

source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# nav2_bringup's bringup_launch.py deliberately does NOT include RViz
# (unlike turtlebot3_navigation2's wrapper, which bundles both together).
# Launch it separately here, in the background, using nav2's own default
# view so you get the map/costmap/planner visualization to actually set
# 2D Pose Estimate and 2D Goal Pose with.
ros2 launch nav2_bringup rviz_launch.py &
RVIZ_PID=$!

ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/map.yaml \
  params_file:=$SCRIPT_DIR/nav2_params.yaml \
  use_sim_time:=False

# Clean up RViz when the main bringup exits/Ctrl+C's
kill $RVIZ_PID 2>/dev/null
