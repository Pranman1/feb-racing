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
#   FEB_BRIDGE   "0" does not start the simulator bridge: the container is then a plain ROS 2
#                workspace that builds src/ and runs FEB_LAUNCH (feb-sim ros)
#   FEB_GUI      "1" starts the web desktop (the gui image only): a screen inside the container
#                served as a web page on port 6080, for rviz2, rqt, turtlesim and the like
#   FEB_APPS     commands to open on that desktop once it is up, separated by ';', e.g.
#                "rviz2 -d /home/autodrive_devkit/rviz/default.rviz; ros2 run turtlesim turtlesim_node"
#   Packages anywhere under /home/autodrive_devkit/src (e.g. the mounted src/stack) are built on start.
set -e
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
cd /home/autodrive_devkit

if find src -name package.xml -not -path "*/autodrive_devkit/*" -not -path "*/feb_tools/*" | grep -q .; then
    colcon build --symlink-install --packages-skip autodrive_roboracer feb_tools
    source install/setup.bash
fi

if [ "$FEB_GUI" = 1 ]; then
    if [ -x /home/autodrive_devkit/desktop.sh ]; then
        source /home/autodrive_devkit/desktop.sh        # sets DISPLAY once the screen is up
    else
        echo "FEB_GUI=1 but this is not the gui image (ghcr.io/pranman1/feb-devkit:gui); no desktop" >&2
    fi
fi

if [ "$FEB_BRIDGE" != 0 ]; then
    if [ -n "$FEB_NOISE" ]; then
        ros2 launch feb_tools bridge_noisy.launch.py $([ "$FEB_NOISE" != 1 ] && echo "$FEB_NOISE") &
    else
        ros2 launch autodrive_roboracer bringup_headless.launch.py &
    fi
fi
if [ "$FEB_FOXGLOVE" != 0 ]; then
    ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0 --log-level warn &
fi
sleep 3
if [ -n "$FEB_LAUNCH" ]; then
    $FEB_LAUNCH &
fi
if [ "$FEB_GUI" = 1 ] && [ -n "$FEB_APPS" ]; then
    # one at a time, a few seconds apart: the window manager puts each new window in free space,
    # which only works if the previous one has drawn itself first (rviz2 takes a while)
    IFS=';' read -ra APPS <<< "$FEB_APPS"
    delay=2
    for app in "${APPS[@]}"; do
        app="${app#"${app%%[![:space:]]*}"}"              # trim leading blanks
        [ -n "$app" ] && (sleep $delay; $app) &
        delay=$((delay + 6))
    done
fi
wait
