# Plan: the devkit as a general ROS 2 environment

Status: done on 2026-10-01; the how-to is `docs/devkit.md`. Nothing here changed how `feb-sim run`
works; the simulator and the league contract stay exactly as they were.

## What the devkit is now

The image `ghcr.io/pranman1/feb-devkit` is ROS 2 Humble (`ros-base`, no GUI tools) with the
simulator bridge pre-built, our Python libraries, and an entrypoint script. The container is
disposable (`--rm`): every start builds whatever is in the mounted `stack/` folder, starts the
bridge, starts the Foxglove WebSocket, and runs the command in `FEB_LAUNCH`. Code lives on the
host in `feb-racing/stack/`, mounted in; nothing of yours is inside the image.

Three lines of `devkit/entrypoint.sh` make it "for the simulator": start the bridge, start
foxglove_bridge, run `FEB_LAUNCH`. Everything else is general.

## What changes

1. **Entrypoint switches** (`devkit/entrypoint.sh`)
   - `FEB_BRIDGE=0`: do not start the simulator bridge. The container then only builds `src/`
     and runs `FEB_LAUNCH`, or sits as a plain ROS box when that is empty.
   - `FEB_APPS`: commands to run once the web desktop is up, one window each (gui image only).
   - Defaults unchanged: bridge on, Foxglove on, `FEB_APPS` empty.

2. **A second image tag** (`devkit/Dockerfile.gui`, pushed as `feb-devkit:gui`)
   - `FROM ghcr.io/pranman1/feb-devkit`, plus `ros-humble-desktop` (rviz2, rqt, turtlesim and
     the rest of the desktop bundle), Xvfb, a light window manager, a VNC server and noVNC.
   - The container serves a Linux desktop as a web page. Windows run inside the container;
     the browser shows them. No X server on the laptop, same on Linux, Mac and Windows.
   - Kept as a separate tag because the desktop bundle adds about 1.5 GB that racing does not need.
   - Expect remote-desktop lag and no GPU: fine for rviz, rqt and turtlesim; the simulator stays native.

3. **A launcher subcommand** (`feb-sim ros`, defined in `feb-sim` next to `run`)
   - Starts the container with no bridge and no simulator app, mounts `stack/` (or `--src <folder>`),
     and gives a shell.
   - `--gui` uses the `gui` tag, maps the desktop port, opens the browser, and opens rviz with a
     saved layout.
   - `--apps "..."` adds windows, e.g. `turtlesim`.
   - `--launch "ros2 launch ..."` runs something instead of the shell.
   - Environment variables as for `run`: `FEB_DEVKIT_NAME`, `FEB_DEVKIT_PORT`, `FEB_FOXGLOVE_PORT`,
     and a new `FEB_VNC_PORT` (default 6080, noVNC's convention, nothing more).

4. **A saved rviz layout** in `devkit/rviz/` so rviz opens already showing the usual topics.

5. **`docs/devkit.md`**: a one-page how-to for members.

## How it looks to a member

| Want | Command | What opens |
|---|---|---|
| Race in the simulator | `./feb-sim run --stack "ros2 launch ..."` | Simulator app and Foxglove, as now |
| Plain ROS devkit | `./feb-sim ros` | A shell inside the container |
| Devkit with a desktop | `./feb-sim ros --gui` | Browser tab at localhost:6080, rviz open |
| turtlesim | `./feb-sim ros --gui --apps turtlesim` | Same desktop, rviz and turtlesim side by side |
| Another terminal | `./feb-sim shell` | One more shell into the running container |
| Stop | `./feb-sim stop` | Container removed |

Two surfaces, always: the browser for anything with a window, the shell for anything typed.
Driving the turtle is `./feb-sim shell` then `ros2 run turtlesim turtle_teleop_key`; the arrow keys
in that shell move the turtle in the browser.

## Files and commands

- Files live on the host. `stack/` on the laptop is the same folder as `src/stack` in the
  container (a bind mount, not a copy), so `ros2 pkg create my_pkg` run in `feb-sim shell` appears
  on the laptop at once, and a file saved in the editor is already inside. There is no sync step.
- Commands run in the container. `ros2`, `colcon`, `rviz2` never have to exist natively on a Mac
  or on Windows; nobody installs ROS on a laptop.
- The entrypoint builds `src/` on every start, so new packages are picked up without extra steps.

## The raw Docker command, for reference

The launcher issues this for `run`; `ros` drops the ports it does not need and adds `-e FEB_BRIDGE=0`:

```
docker run -d --rm --name feb-devkit \
  -p 4567:4567 -p 8765:8765 \
  -e ROS_LOCALHOST_ONLY=1 \
  -v "$(pwd)/stack:/home/autodrive_devkit/src/stack" \
  -v "$(pwd)/tracks:/home/autodrive_devkit/tracks:ro" \
  -e "FEB_LAUNCH=ros2 launch feb_driver drive.launch.py" \
  ghcr.io/pranman1/feb-devkit:latest
```

The host side of each `-v` is the laptop's own path (the launcher fills it in; a Linux path pasted
onto a Mac fails with "mounts denied"). The container side is the only convention the image
carries: the entrypoint looks under `src/` for packages. `--entrypoint bash` switches all of the
automation off and leaves a Humble shell.

## Effort

- Entrypoint switches and the `ros` subcommand: about an hour.
- GUI image: an afternoon, mostly image builds and trying noVNC on a Mac and a Windows laptop.
- Docs and the rviz layout: an hour.

## Decisions still open

- Whether the `gui` tag is worth maintaining. For teaching ROS to new members, yes.
- Whether `feb-sim ros` without `--gui` should open a shell (proposed) or just start and return.
