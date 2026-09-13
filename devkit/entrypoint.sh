#!/bin/bash
# Entry point of the FEB devkit container (same contract as the league's autodrive_devkit.sh):
# everything needed to race starts from here, nothing from ~/.bashrc.
#
#   FEB_LAUNCH   command that starts the racing stack after the bridge, e.g.
#                "ros2 launch feb_driver drive.launch.py". Empty = bridge only (teleop/practice).
#   Packages anywhere under /home/autodrive_devkit/src (e.g. the mounted src/stack) are built on start.
set -e
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
cd /home/autodrive_devkit

if find src -name package.xml -not -path "*/autodrive_devkit/*" | grep -q .; then
    colcon build --symlink-install --packages-skip autodrive_roboracer
    source install/setup.bash
fi

ros2 launch autodrive_roboracer bringup_headless.launch.py &
sleep 3
if [ -n "$FEB_LAUNCH" ]; then
    $FEB_LAUNCH &
fi
wait
