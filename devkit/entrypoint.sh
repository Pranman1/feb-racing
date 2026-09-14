#!/bin/bash
# Entry point of the FEB devkit container (same contract as the league's autodrive_devkit.sh):
# everything needed to race starts from here, nothing from ~/.bashrc.
#
#   FEB_LAUNCH   command that starts the racing stack after the bridge, e.g.
#                "ros2 launch feb_driver drive.launch.py". Empty = bridge only (teleop/practice).
#   FEB_NOISE    sensor noise/dropout, e.g. "range_sigma:=0.03 beam_dropout:=0.05 latency:=0.05"
#                (any value, even "1", enables the defaults of feb_tools/sensor_noise.py)
#   FEB_FOXGLOVE "0" disables the Foxglove bridge (a WebSocket on port 8765 that lets Foxglove
#                on the laptop see every topic live, on any OS, without ROS installed there)
#   Packages anywhere under /home/autodrive_devkit/src (e.g. the mounted src/stack) are built on start.
set -e
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
cd /home/autodrive_devkit

if find src -name package.xml -not -path "*/autodrive_devkit/*" -not -path "*/feb_tools/*" | grep -q .; then
    colcon build --symlink-install --packages-skip autodrive_roboracer feb_tools
    source install/setup.bash
fi

if [ -n "$FEB_NOISE" ]; then
    ros2 launch feb_tools bridge_noisy.launch.py $([ "$FEB_NOISE" != 1 ] && echo "$FEB_NOISE") &
else
    ros2 launch autodrive_roboracer bringup_headless.launch.py &
fi
if [ "$FEB_FOXGLOVE" != 0 ]; then
    ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0 --log-level warn &
fi
sleep 3
if [ -n "$FEB_LAUNCH" ]; then
    $FEB_LAUNCH &
fi
wait
