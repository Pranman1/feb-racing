# The devkit as a ROS 2 workspace

The devkit container is ROS 2 Humble with the simulator bridge inside. `feb-sim run` uses it to
race; `feb-sim ros` uses the same container with the bridge switched off, as a plain ROS 2
workspace for learning ROS, trying packages, or running tools that have nothing to do with the
simulator. Nothing has to be installed on the laptop beyond Docker and a browser.

## Two surfaces

- **The browser** shows anything with a window. `--gui` starts a desktop *inside* the container
  and serves it as a web page at `http://localhost:6080`; rviz2, rqt and turtlesim open there. The
  same on Linux, Mac and Windows, no X server on the laptop.
- **The shell** is for anything typed. `feb-sim shell` opens a terminal inside the running
  container with ROS sourced; open as many as you like.

## Commands

| Want | Command |
|---|---|
| A ROS 2 shell, nothing started | `./feb-sim ros` |
| A desktop in the browser with rviz2 open | `./feb-sim ros --gui` |
| rviz2 and turtlesim | `./feb-sim ros --gui --apps turtlesim` |
| rqt as well | `./feb-sim ros --gui --apps "turtlesim rqt"` |
| Any window, e.g. a Qt tool | `./feb-sim ros --gui --apps "ros2 run my_pkg my_gui"` |
| Run a launch file instead of a shell | `./feb-sim ros --launch "ros2 launch my_pkg thing.launch.py"` |
| Mount a folder other than `stack/` | `./feb-sim ros --src ~/code/my_ws/src` |
| Another terminal | `./feb-sim shell` |
| The log | `./feb-sim logs -f` |
| Stop | `./feb-sim stop` |

`--gui` uses a second image, `ghcr.io/pranman1/feb-devkit:gui`, which is the devkit plus
`ros-humble-desktop` and the web desktop (about 1.5 GB more). It is pulled the first time it is
used; `feb-sim update` keeps it current for those who have it.

## turtlesim in two minutes

```
./feb-sim ros --gui --apps turtlesim        # a browser tab opens: rviz2 and the turtle
./feb-sim shell                             # in a second terminal
ros2 run turtlesim turtle_teleop_key        # arrow keys here move the turtle in the browser
```

Other terminals, `ros2 topic list`, `ros2 topic echo /turtle1/pose`, and so on, the usual way.

## Your own packages

Files live on the laptop; commands run in the container. `stack/` on the laptop *is*
`/home/autodrive_devkit/src/stack` inside (a bind mount, not a copy), so:

```
./feb-sim shell
cd src/stack
ros2 pkg create my_pkg --build-type ament_python --node-name hello
colcon build --packages-select my_pkg && source install/setup.bash
ros2 run my_pkg hello
```

The package appears in `stack/my_pkg` on the laptop at once; edit it there. The container builds
everything under `src/` on every start, so the next `feb-sim ros` has it built already.

## What is and is not in the images

| | base (`:latest`) | gui (`:gui`) |
|---|---|---|
| ROS 2 Humble | `ros-base` | `ros-base` + `ros-humble-desktop` |
| Simulator bridge, Foxglove bridge | yes | yes |
| rviz2, rqt, turtlesim | no | yes |
| Web desktop (Xvfb, fluxbox, x11vnc, noVNC) | no | yes |

The desktop has no GPU: rviz2 renders in software, which is fine for maps and plots and would not
be for a 3D simulator, which is why the simulator stays a native app.

## Ports and overrides

| Variable | Default | What |
|---|---|---|
| `FEB_VNC_PORT` | 6080 | the web desktop on the laptop |
| `FEB_FOXGLOVE_PORT` | 8765 | the Foxglove WebSocket |
| `FEB_DEVKIT_NAME` | feb-devkit | container name, to run two side by side |
| `FEB_DEVKIT_GUI_IMAGE` | `ghcr.io/pranman1/feb-devkit:gui` | which gui image |

## Under the hood

`feb-sim ros` is one `docker run`: the mount, `FEB_BRIDGE=0`, and with `--gui` the port 6080 and
`FEB_GUI=1` plus `FEB_APPS` (the windows to open, `;`-separated). The entrypoint builds `src/`,
starts the desktop if asked, skips the bridge, runs `FEB_LAUNCH` if given, then opens each app.
`devkit/entrypoint.sh` and `devkit/desktop.sh` are the whole of it.
