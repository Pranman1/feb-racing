#!/bin/bash
# The web desktop of the gui image: a screen with no monitor (Xvfb), a window manager, a VNC
# server on it and noVNC, which serves that screen as a web page on port 6080. Sourced by the
# entrypoint when FEB_GUI=1; everything started afterwards inherits DISPLAY.
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1          # no GPU in the container: rviz2 renders in software
export QT_X11_NO_MITSHM=1
Xvfb :1 -screen 0 "${FEB_SCREEN:-1600x900x24}" -nolisten tcp >/dev/null 2>&1 &
for _ in $(seq 1 50); do xdpyinfo -display :1 >/dev/null 2>&1 && break; sleep 0.1; done
mkdir -p ~/.fluxbox                     # a plain dark root, or fluxbox pops up a wallpaper complaint
grep -q rootCommand ~/.fluxbox/init 2>/dev/null || echo 'session.screen0.rootCommand: hsetroot -solid "#1a1612"' >> ~/.fluxbox/init
fluxbox >/dev/null 2>&1 &
# x11vnc has been seen to exit on a client disconnect (a browser tab closing or sleeping) despite
# -forever, which leaves the page unable to connect: keep restarting it, and keep its log
(set +e                                   # the entrypoint's set -e would end the loop with x11vnc
 while true; do
    x11vnc -display :1 -nopw -forever -shared -noxdamage -rfbport 5900 -localhost >>/tmp/x11vnc.log 2>&1
    echo "x11vnc exited ($?), restarting" >>/tmp/x11vnc.log; sleep 1
done) &
websockify --web=/usr/share/novnc 6080 localhost:5900 >/dev/null 2>&1 &
echo "web desktop on port 6080 (open http://localhost:6080/vnc.html?autoconnect=1&resize=scale)"
