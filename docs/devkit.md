# The FEB devkit

The devkit is where your ROS 2 code runs. It is a Docker image, `ghcr.io/pranman1/feb-devkit`,
with ROS 2 Humble, the simulator bridge and our Python libraries inside, and nothing of yours.
Your code stays on your laptop in `feb-racing/stack/`; the container mounts that folder, builds
it when it starts, and is thrown away when it stops. The simulator is a separate native app for
your OS that talks to the devkit over a port. One launcher, `./feb-sim`, starts both.

## Setup, once

```
git clone git@github.com:Pranman1/feb-racing.git && cd feb-racing
./feb-sim setup --team "Your Team"      # checks Docker, pulls the image, downloads the simulator app
./feb-sim update                         # later: git pull, newer images, newer app, all in one
```

Docker Desktop must be running. On a Mac, keep the clone under your home folder (`/Users/...`);
Docker can only mount folders from there. Foxglove (foxglove.dev) is optional and recommended.

## Two ways to start it

| Want | Command |
|---|---|
| Race: devkit + your stack + simulator + Foxglove | `./feb-sim run --track loop --stack "ros2 launch feb_driver drive.launch.py"` |
| Drive by hand, no stack | `./feb-sim run --track loop` |
| A plain ROS 2 workspace, no simulator, a shell | `./feb-sim ros` |
| The same with a desktop in the browser (rviz2 open) | `./feb-sim ros --gui` |
| Plus turtlesim, rqt, or any window | `./feb-sim ros --gui --apps "turtlesim rqt"` |
| Run a launch file instead of a shell | `./feb-sim ros --launch "ros2 launch my_pkg x.launch.py"` |
| Mount a folder other than `stack/` | `./feb-sim ros --src ~/code/my_ws/src` |

And while it runs:

| | |
|---|---|
| `./feb-sim shell` | another terminal inside the container, ROS sourced, as many as you like |
| `./feb-sim logs -f` | the build, the bridge and everything your nodes print |
| `./feb-sim stop` | stop and remove the container (closing the simulator window does this too) |
| `./feb-sim practice --stack "..."` | a scored run under competition rules |
| `./feb-sim submit` | post the newest practice result to the leaderboard |

`run` has more flags: `--mode manual|autonomous`, `--cars N`, `--opponent slow|normal|fast|map`,
`--camera "God's Eye"|Trackcam|"Driver's Eye"`, `--look visual|simple`, `--headless`, `--noise`,
`--lidar-hz`, `--fps`, `--camera-hz`, `--no-foxglove`. `./feb-sim run --help` lists them.

## Inside the container

```
/home/autodrive_devkit/          where a shell starts
├── install/                     the built bridge and your packages
├── tracks/                      <- your tracks/ folder (read-only)
└── src/
    ├── autodrive_devkit/        the simulator bridge (in the image)
    ├── feb_tools/               house driver, sensor noise (in the image)
    └── stack/                   <- your stack/ folder (read-write)
```

The two arrows are bind mounts: the same folder seen from two places, not copies. Everything
else is the image and vanishes on stop.

## Your code

Files live on the laptop, commands run in the container, windows show in the browser.

```
./feb-sim shell
cd src/stack
ros2 pkg create my_pkg --build-type ament_python --node-name hello
cd /home/autodrive_devkit
colcon build --packages-select my_pkg && source install/setup.bash
ros2 run my_pkg hello
```

The package appears at `stack/my_pkg` on the laptop at once; edit it there in your editor. The
build uses `--symlink-install`, so Python edits take effect on the next `ros2 run` without
rebuilding. Every start of the container builds whatever is under `src/`, so the next
`feb-sim run` or `feb-sim ros` has it ready. A node's output goes to the container log
(`feb-sim logs -f`) when the entrypoint launched it, or to your shell when you launched it there.

`ros2 pkg create` must be run inside `src/stack`; run it anywhere else and the files land in the
container's own filesystem and are lost on stop.

## The desktop in the browser

`--gui` uses a second image, `feb-devkit:gui` (the devkit plus `ros-humble-desktop` and a web
desktop; about 2.7 GB more, downloaded the first time). A screen inside the container is served
as a web page at `http://localhost:6080`; rviz2, rqt, turtlesim and anything else with a window
open there. Same on Linux, Mac and Windows, no X server on the laptop. It has no GPU: fine for
rviz and plots, which is why the simulator stays a native app.

turtlesim in two minutes:

```
./feb-sim ros --gui --apps turtlesim     # browser tab opens: rviz2 and the turtle
./feb-sim shell                          # second terminal
ros2 run turtlesim turtle_teleop_key     # arrow keys here move the turtle in the browser
```

Anything with a window that you start from `./feb-sim shell` (`rqt_graph &`, `rviz2 &`) opens on
that desktop too.

rviz2 opens with the layout in `devkit/rviz/default.rviz` (the cone racer's topics, an empty grid
otherwise). Because rviz subscribes to them, `ros2 topic list` shows `/feb/map`, `/feb/pose` and
friends even with nothing publishing; that is normal.

New to ROS 2? `docs/lab-ros-onboarding.md` is a two-session lab (packages, topics, services,
turtlesim) that runs entirely in this desktop.

## Looking at topics without the desktop

Foxglove: `feb-sim run` opens it on the devkit automatically if installed. By hand: Open
connection -> Foxglove WebSocket -> `ws://localhost:8765`. Any OS, no ROS on the laptop.

## Ports and overrides

| Variable | Default | What |
|---|---|---|
| `FEB_DEVKIT_PORT` | 4567 | the bridge; the simulator connects here |
| `FEB_FOXGLOVE_PORT` | 8765 | the Foxglove WebSocket |
| `FEB_VNC_PORT` | 6080 | the web desktop |
| `FEB_DEVKIT_NAME` | feb-devkit | container name; change it to run two side by side |
| `FEB_DEVKIT_IMAGE` | `ghcr.io/pranman1/feb-devkit:latest` | the image |
| `FEB_SIM_APP` | the installed app | another simulator binary |

## When something is wrong

| Symptom | Cause, fix |
|---|---|
| a package created in the container is read-only on a Linux laptop | the container runs as root: `sudo chown -R $USER stack` once |
| `mounts denied` on a Mac | the clone is outside `/Users`; move it, or add the folder in Docker Desktop -> Resources -> File Sharing |
| `port is already allocated` | another devkit is running: `./feb-sim stop`, or set `FEB_DEVKIT_PORT` |
| the simulator says not connected | the bridge is not up yet: it starts after the build, give it a minute, watch `feb-sim logs -f` |
| my node is not there | it did not build: `feb-sim logs -n 100` shows the colcon errors |
| the desktop page cannot connect | reload; the screen server restarts itself. Still nothing: `docker exec feb-devkit tail /tmp/x11vnc.log` |
| the page is tiny or has scrollbars | use the URL the launcher prints, it ends in `resize=scale` |
| a Linux path in a pasted `docker run` | paths are per machine; use `./feb-sim`, which fills them in |

## Under the hood

`feb-sim run` issues one `docker run`:

```
docker run -d --rm --name feb-devkit \
  -p 4567:4567 -p 8765:8765 \
  -e ROS_LOCALHOST_ONLY=1 \
  -v "$(pwd)/stack:/home/autodrive_devkit/src/stack" \
  -v "$(pwd)/tracks:/home/autodrive_devkit/tracks:ro" \
  -e "FEB_LAUNCH=ros2 launch feb_driver drive.launch.py" \
  ghcr.io/pranman1/feb-devkit:latest
```

`-d` runs it in the background (the log is `feb-sim logs`); `--rm` removes it on stop; `-p`
publishes ports; `-v` mounts folders; `-e` passes settings to the entrypoint. The entrypoint,
`devkit/entrypoint.sh`, sources ROS, builds `src/`, starts the bridge (unless `FEB_BRIDGE=0`),
starts the Foxglove bridge (unless `FEB_FOXGLOVE=0`), starts the desktop if `FEB_GUI=1`, runs
`FEB_LAUNCH`, then opens each command in `FEB_APPS`. `feb-sim ros` is the same command with
`FEB_BRIDGE=0`, no 4567, and with `--gui` the `gui` image, port 6080, `FEB_GUI=1` and `FEB_APPS`.
`--entrypoint bash` on the raw command switches all of that off and leaves a Humble shell.

The images are built by CI for amd64 and arm64 whenever `devkit/` changes on `main`; nobody
pushes them by hand. A team's competition image is `docker build -f starter_kit/Dockerfile stack/`,
which starts `FROM` the devkit and copies `stack/` in: that is how code in a mount becomes an image.
