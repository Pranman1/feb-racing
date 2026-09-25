# Starter kit and learning path

`starter_kit/feb_driver` is a complete ROS 2 package that drives a lap on any track. You
copy it, make it faster, and race it. Every stage below ends with a number on the leaderboard.

## Setup (once)

```bash
git clone https://github.com/Pranman1/feb-racing && cd feb-racing
./feb-sim setup --team "Your Team"      # Docker check, devkit image, simulator app for your OS
cp -r starter_kit/feb_driver stack/     # your copy; stack/ is mounted into the container
./feb-sim run                           # drive with the arrow keys first
./feb-sim run --stack "ros2 launch feb_driver drive.launch.py"   # the starter driver
```

Requirements: Docker Desktop (Mac/Windows) or docker.io (Linux), Python 3, the GitHub CLI for
`feb-sim submit`, and [Foxglove](https://foxglove.dev/download) (sign in once) to see your car's data. Your code stays on your laptop and is built inside the container on start.

## Stage 0: read the car

Topics (namespace `/autodrive/roboracer_1/`): `lidar` (1080 beams, 270°, 0.06 to 10 m, 40 Hz),
`imu`, `left_encoder` / `right_encoder` (wheel angle in rad, 0.059 m radius), `steering` and
`throttle` feedback. Commands: `steering_command` in [-1, 1] = ±0.5236 rad, `throttle_command`
in [-1, 1] where 0 is a hard brake. Steering lags about 0.25 s and slews at 3.2 rad/s.
`./feb-sim shell` gives you a ROS 2 shell: `ros2 topic hz`, `ros2 topic echo`. To *see* the data
(lidar in 3D, the camera, plots of throttle and steering) install [Foxglove](https://foxglove.dev/download):
`feb-sim run` opens it on your car (`ws://localhost:8765`). It also opens the bags under `runs/`.

## Stage 1: reactive driving (follow the gap)

Read `feb_driver/reactive_driver.py` (about 170 lines). Tune `config/driver.yaml`: bubble, gap
FOV, the deep-beam weight, speed limits, braking margin, the filter time constants. Note the
throttle law: feed-forward plus a trim bounded to half of it. Throttle 0 is a hard brake here,
so a textbook PI speed loop surges and stops; keep the command away from zero. Score yourself: `./feb-sim practice --stack "ros2 launch feb_driver drive.launch.py"`,
post with `./feb-sim submit`. Target: 10 clean laps on `loop` under 100 s.

## Stage 2: system identification

`ros2 launch feb_driver sysid.launch.py` drives throttle steps, a coast and a chirp while
recording a bag into `stack/sysid_bag`. Fit it: `python3 tools/sysid_fit.py stack/sysid_bag`
(inside the container) gives the steady-state gain (m/s per throttle), the idle-brake
deceleration and the throttle and steering lags. Put the gain into `speed_per_throttle`.
Expect the tyres to be a knife edge: lateral grip peaks at 1 % slip and halves by 10 %.

## Stage 3: localisation and a map

Build a map of the track from lidar (scan matching, or SLAM Toolbox in the container), then
localise on it at race time. Ground-truth `ips`/`odom`/`tf` may be used to *check* your
estimate during development, never at race time.

## Stage 4: raceline and controller

Optimise a minimum-curvature line on your map with a speed profile from your sysid numbers,
and track it (pure pursuit, then MAP or MPC). This is where lap times halve.

## The cone track: two more packages

`loop_cones` has no walls, only blue cones on the left and yellow on the right, so the
follow-the-gap driver has nothing to follow. Two packages in `starter_kit/` are the answer, and
between them they are the FSAE car's pipeline at small scale.

### `feb_cone_driver`: the baseline (one file)

```bash
cp -r starter_kit/feb_cone_driver stack/
./feb-sim run --track loop_cones --look visual --stack "ros2 launch feb_cone_driver drive.launch.py"
```

Per lidar scan: small clusters of returns are cone candidates; the camera's blue and yellow
blobs give them a colour by image column (camera pose, focal length and the image's lag behind
the scan are calibrated numbers in the yaml); a cone keeps its colour for a while after it leaves
the camera's view, and a cone right beside the car takes its side's colour; the blue and yellow
cones are chained along the track (the chain step follows the spacing of the cones in view,
so 1 m and 2.5 m layouts both work), each blue cone pairs with the yellow cone across the
track to its right, the midpoints are the centreline, and pure pursuit follows it at 1 to
1.2 m/s with the starter kit's throttle law. If the lidar scene stops moving while the
throttle is on, or the car stops with nothing left to follow (nosed out of a hairpin), it
backs up for a second and tries again. It laps `loop_cones` indefinitely (7 laps in 240 s in
testing) and `spielberg_cones` (2 laps in 12 minutes, no cone touched) and every number is in
`config/cone_driver.yaml`. Use `--look visual`: the camera thresholds are tuned for the dressed
scene, and the bare one is too dark for them.

### `feb_cone_racer`: map, raceline, MPC

```bash
cp -r starter_kit/feb_cone_racer stack/
./feb-sim run --track loop_cones --look visual --stack "ros2 launch feb_cone_racer race.launch.py"
# in another terminal, once, while the devkit is running:
starter_kit/feb_cone_racer/install_deps.sh      # scipy + CasADi into stack/.pydeps
```

Lap 1 is driven by the baseline follower while **GraphSLAM** (the formulation of the team's FSAE
`graphslam_global`, poses and cones as a sparse linear least-squares problem, ICP data
association with colour votes, a wide-net loop closure at the orange start gate) builds the
map from wheel odometry, IMU heading and the coloured cones. Back at the start the map is
frozen and the track is built: cone colours are repaired by which side of the car's own
mapping-lap path they lie on (the follower keeps that path near the middle, so it beats a
camera mislabel), the boundaries are ordered with a step that follows the cone spacing and
survives a missing cone, a centreline is sampled and its width re-measured (a track's width
is nearly constant, so where the boundaries collapse the mapping path fills in), then a
**minimum-curvature raceline** is solved as a sparse QP (CasADi, 10 ms for 1300 points) and
a **speed profile** laid over it from lateral, acceleration and braking limits. From then on
the car localises against the map by ICP on every lidar cone (coloured or not), finds itself
again anywhere on the map from the IMU heading if the match is lost, and a **nonlinear MPC**
(dynamic bicycle model with Pacejka-style tyres and the sysid longitudinal model, direct
multiple shooting, IPOPT, about 10 ms a solve) tracks the raceline. Without CasADi the node
still maps and plans, and pure pursuit drives the raceline. Debug topics: `/feb/cones`,
`/feb/map`, `/feb/raceline`, `/feb/pose`, `/feb/mpc_prediction`. Parameters, including the
vehicle model, are in `config/racer.yaml`. In testing it maps `loop_cones` in 31 s and then
laps it in about 23 s, and maps `spielberg_cones` (342 m, 292 cones) in 5 minutes and then
laps it in about 213 s, without touching a cone; `race_speed_scale` is the knob to turn up.

## Stage 5: realism

`./feb-sim run --noise 1` adds lidar range noise and dropouts; `--noise "range_sigma:=0.05 latency:=0.05"`
sets them; `--lidar-hz 10` runs the lidar at the rate the real bridge often delivers.
`./feb-sim run --stack "..." --opponent normal` races you against the house racer in car two
(`slow`, `normal`, `fast`, or `map`), through the same 10 Hz proxy the competitions use.

## Submitting for competitions

```bash
docker build -t <you>/feb-racer:v1 stack/       # FROM ghcr.io/pranman1/feb-devkit, see starter_kit/Dockerfile
docker push <you>/feb-racer:v1
```
Add your team and image to `submissions.yaml` in a pull request. The image must start the
bridge and your stack from `/home/autodrive_devkit.sh` (the starter Dockerfile does).
