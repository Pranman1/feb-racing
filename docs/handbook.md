# FEBAUTO Racing handbook

One document for both roles. Printable manuals: `docs/manual/competitor.pdf` and `docs/manual/organiser.pdf` (LaTeX source alongside). Detail pages: `starter-kit.md`, `tracks.md`, `competition.md`,
`head-to-head.md`, `semester.md`, `build.md`, `unity-licence.md`.

## What the pieces are

| Piece | What it does | Who touches it |
|---|---|---|
| FEB Simulator (native app) | Simulates the 1:10 RoboRacer car on a track folder; talks to a bridge over a WebSocket on port 4567 | everyone runs it, nobody edits it |
| Devkit container | ROS 2 Humble + the AutoDRIVE bridge; turns the WebSocket into ROS 2 topics and back; also hosts the member's code | members' code runs inside it |
| `stack/` | The member's ROS 2 packages, mounted into the container and built on start | members |
| `tracks/` | Public tracks: PNG + YAML in, `track.json` out; shipped inside the app | organiser adds, members use |
| `feb-sim` | The member launcher: setup, run, practice, submit | members |
| `feb-race` | The organiser harness: events, nightly reruns, verification, head-to-head brackets | organiser |
| `results/`, `events/`, `submissions.yaml` | Data behind the leaderboard site | harness writes, members' pull requests add |
| Site | https://pranman1.github.io/feb-racing/ built by GitHub Actions on every push | read-only |

Data flow while driving:

```
lidar, imu, encoders, camera, lap/collision counts  -->  bridge  -->  /autodrive/roboracer_1/*  -->  your node
your node  -->  /autodrive/roboracer_1/steering_command, throttle_command  -->  bridge  -->  simulator
```

The car takes exactly two inputs: steering in [-1, 1] (full left to full right, ±0.5236 rad)
and throttle in [-1, 1] (0 is a hard brake). Everything else is a sensor.

## Competitor: from zero to a leaderboard entry

1. **Install** (once). Docker Desktop (Mac/Windows) or `docker.io` (Linux), Python 3, the GitHub CLI.
   ```bash
   git clone https://github.com/Pranman1/feb-racing && cd feb-racing
   ./feb-sim setup --team "Your Team"     # pulls the devkit image, downloads the app for your OS
   ./feb-sim run                          # manual driving: arrow keys; menu top-left
   ```
2. **Get code.** `cp -r starter_kit/feb_driver stack/`. The whole driver is
   `stack/feb_driver/feb_driver/reactive_driver.py` (140 lines). Its tuning knobs are
   `stack/feb_driver/config/driver.yaml`. `stack/` is yours: add packages, nodes, files.
3. **Run it.** `./feb-sim run --stack "ros2 launch feb_driver drive.launch.py"`. The launcher
   starts the container (which builds your packages), then the simulator, connected. Edit,
   rerun. `./feb-sim shell` gives a ROS 2 shell in the container (`ros2 topic echo`, `rqt`).
   Options: `--track porto`, `--noise 1`, `--lidar-hz 10`, `--opponent normal`, `--camera Trackcam`.
   In the app: the menu's "Track" and "Cars" buttons cycle tracks and put more cars on the grid.
4. **Score yourself.** `./feb-sim practice --stack "..."`: warm-up plus 10 timed laps, 10 s per
   collision, DNF after 300 s. The result and a bag land in `runs/<id>/`.
5. **Post it.** `./feb-sim submit`: opens a pull request with the result file; a check validates
   it and merges; you appear on that track's practice board as *unverified* until the organiser
   reruns your image.
6. **Enter competitions.** Build your image and register it:
   ```bash
   docker build -t <dockerhub-user>/feb-racer:v1 -f starter_kit/Dockerfile stack/
   docker push <dockerhub-user>/feb-racer:v1
   ```
   Add your team, GitHub logins and image to `submissions.yaml` in a pull request. Competitions
   and nightly reruns pull that image; the entrypoint starts your stack automatically.

Rules you must respect: no `ips`, `odom`, `tf`, lap or collision counters or `reset_command`
at race time (training and debugging only). The harness audits which nodes subscribe to them
and flags the result. Everything must start from the container's entrypoint, not `.bashrc`.

## Organiser: running the programme

1. **Tracks.** Public ones live in `tracks/` and ship inside the app. Make one in two commands
   (`docs/tracks.md`): waypoints YAML -> `track_design.py` -> `track_build.py`. Keep competition
   tracks in a private folder outside the repo until their event; afterwards move them into
   `tracks/`, commit, and cut a new app release (`feb-sim/build.sh all`, `docs/build.md`).
2. **Events.** One YAML per event in `events/` (title, date, mode, track path, attempts, rules).
   Pushing it lists the event on the site.
3. **Race day (time-attack).**
   ```bash
   ./feb-race event 2026-10-qualification     # every registered image, N attempts each, standings printed
   git add results && git commit -m "qualification results" && git push
   ```
   Each attempt: pull image, start container, start the headless simulator on the secret track,
   probe laps and collisions inside the container, record a bag, tear down, write `result.json`.
4. **Finals (head-to-head).** `./feb-race bracket 2026-12-final --seed-from 2026-10-qualification`:
   knockout races of two cars, each car in its own container behind the proxy. Results carry
   race id and position; the site shows the bracket.
5. **Between events.** `./feb-race nightly` reruns every registered image on every practice
   track and posts verified results; `./feb-race verify <result.json>` checks one self-reported
   entry. Put nightly in cron and push `results/` afterwards.
6. **Site and images.** Nothing to do: pushes to `main` rebuild the site and, when `devkit/`
   changes, the devkit image (`ghcr.io/pranman1/feb-devkit`).

## Where things are on the organiser PC

| Item | Path |
|---|---|
| Repos | `~/FEB/feb-racing` (this), `~/FEB/feb-sim` (Unity fork, branch `feb`) |
| Simulator players | `~/FEB/feb-sim/Builds/{linux,mac,windows}`; Linux also linked at `~/.feb-sim/app/linux` |
| Unity editor | `~/Unity/Hub/Editor/2022.3.52f1` (licensed via Hub sign-in) |
| Run folders | `~/FEB/feb-racing/runs/<id>/` (result.json, sim.log, container.log, bag) |
| Ports | 4567 member devkit, 4568+ harness containers (one per car), 4580 harness proxy, 4565/4566 launcher proxy and opponent |

## Frequently hit

- *Port 4567 in use*: another bridge container is running (`docker ps`; `./feb-sim stop`).
- *Car does not move*: the stack did not start (`./feb-sim shell`, `ros2 node list`), or the
  mode is Manual (the launcher picks Autonomous whenever `--stack` is given; the menu button toggles it).
- *Bridge rate*: with a window open the simulator exchanges data at about 10 Hz on Linux
  (each exchange reads the cameras back from the GPU); headless it is 20 to 40 Hz. Drivers must
  cope with 10 Hz: the starter kit and the house driver do, a plain PI speed loop does not.
- *Nothing drives*: correct. Without `--stack` the launcher starts no driver and no opponent;
  car one answers the keyboard in Manual mode, and extra cars are props until `--opponent`
  (or the harness) puts a driver behind them. Car one is always the ego car: the mode button
  and the Driver's Eye camera both belong to it.
- *No window on the organiser PC after a driver update*: reboot (GPU driver mismatch).
- *Cones or walls missing*: the track folder has no `track.json`; run `tools/track_build.py`.
