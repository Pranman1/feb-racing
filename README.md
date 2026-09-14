# FEB Racing

Autonomous sim-racing for Formula Electric at Berkeley: a 1:10-scale racing simulator
(fork of [AutoDRIVE](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE)), a ROS 2 devkit,
custom tracks, a competition harness and a leaderboard.

The RoboRacer vehicle model is the official one, unchanged, so what you tune here transfers
to the RoboRacer Sim Racing League and to the real 1:10 cars.

| Piece | Where |
|---|---|
| Simulator (Unity fork, native Mac/Windows/Linux apps) | `../feb-sim` (separate repo) |
| Devkit image (ROS 2 Humble + AutoDRIVE bridge) | `devkit/` |
| Tracks (PNG in, geometry out) | `tracks/`, `tools/track_design.py`, `tools/track_build.py` |
| One-command launcher | `./feb-sim` |
| Organiser harness (time-attack, nightly reruns, head-to-head brackets) | `./feb-race`, `febrace/` |
| Leaderboard site (GitHub Pages) | `site/`, `results/`, `events/`, `submissions.yaml` |
| Starter kit (reactive driver, sysid probe, submission Dockerfile) | `starter_kit/` |
| Docs | `docs/`: build, tracks, competition, starter-kit, head-to-head, semester, unity-licence |

**Start with [docs/handbook.md](docs/handbook.md)**: the whole flow for competitors and the organiser.

## Quick start (members)

```bash
git clone https://github.com/Pranman1/feb-racing && cd feb-racing
./feb-sim setup                 # checks Docker, pulls the devkit, downloads the simulator app
./feb-sim run                   # simulator + bridge, drive with the keyboard
./feb-sim run --stack "ros2 launch feb_driver drive.launch.py"   # your stack (see starter_kit/)
./feb-sim practice --stack "..."                                 # scored attempt, then ./feb-sim submit
```

Your ROS 2 packages live in `stack/` and are mounted into the container and built on start.

## Adding a track (organiser)

```bash
mkdir tracks/mytrack && $EDITOR tracks/mytrack/design.yaml     # waypoints + width
tools/track_design.py tracks/mytrack/design.yaml               # -> map.png
tools/track_build.py tracks/mytrack                            # -> track.json + preview.png
./feb-sim run --track mytrack
```

Any white-corridor PNG works as `map.png`, including SLAM occupancy grids of real tracks
(`tracks/porto` is the legacy RoboRacer Porto map). See `docs/tracks.md`.

## Rules

Same as the RoboRacer Sim Racing League: ground-truth topics (`ips`, `odom`, `tf`, lap and
collision counters, `reset_command`) may be used for training and debugging but never at race
time; 10 s penalty per collision.

## Credits

Built on the AutoDRIVE Ecosystem (BSD-2-Clause). Please cite:
Samak et al., "AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem for
Autonomous Driving Research & Education", Robotics 2023, doi:10.3390/robotics12030077, and
Samak et al., "AutoDRIVE Simulator: A Simulator for Scaled Autonomous Vehicle Research and
Education", CCRIS 2021, doi:10.1145/3483845.3483846.
