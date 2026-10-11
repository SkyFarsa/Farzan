#!/bin/bash
set -e

# --- Source ROS 2 Humble ---
source /opt/ros/humble/setup.bash

# --- Source your workspace ---
source ~/ros2_ws/install/setup.bash

# --- Launch Cartographer SLAM ---
ros2 launch my_robot_cartographer cartographer.launch.py &
CARTO_PID=$!

sleep 5

# --- Launch RViz ---
RVIZ_CONFIG=~/ros2_ws/src/my_robot_cartographer/rviz/my_robot_cartographer.rviz

if [ -f "$RVIZ_CONFIG" ]; then
  rviz2 -d "$RVIZ_CONFIG"
else
  rviz2
fi

# --- After RViz closes: save the map ---
echo "Saving map to ~/map ..."
ros2 run nav2_map_server map_saver_cli -f ~/map

# --- Kill Cartographer gracefully ---
kill $CARTO_PID 2>/dev/null || true
wait $CARTO_PID 2>/dev/null || true

echo "Map saved to: ~/map.yaml  and  ~/map.pgm"
