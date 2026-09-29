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

### `feb_cone_racer` and `feb_cone_ordering`: not released to members yet

The two packages below are the full pipeline (map, the team's cone ordering, raceline, MPC).
They live in `starter_kit/` for the organiser and the core team and are not in the member
manuals; how and when they reach members is still to be decided.

### `feb_cone_racer`: map, raceline, MPC

```bash
cp -r starter_kit/feb_cone_racer starter_kit/feb_cone_ordering stack/
./feb-sim run --track loop_cones --look visual --stack "ros2 launch feb_cone_racer race.launch.py"
# in another terminal, once, while the devkit is running:
starter_kit/feb_cone_racer/install_deps.sh      # scipy + CasADi into stack/.pydeps
```

Lap 1 is driven the way the car does it: the map grows with every keyframe, the team's cone
ordering runs live on it and the car follows the local path its rungs describe, as far as the
rungs still look like the track (past the mapped frontier the ordering's path fans out or
zigzags, and those rungs are not trusted); the reactive follower takes over where the trusted
path is shorter than 2 m or the rungs are stale, and drives when the ordering package is not in
the stack. The reactive follower walks the blue and yellow cone chains in view every 0.3 m and
aims at the point exactly one lookahead along the middle, so it turns in at a corner rather than
a cone length past it. Meanwhile **GraphSLAM** (the formulation of the team's FSAE
`graphslam_global`, poses and cones as a sparse linear least-squares problem, ICP data
association with colour votes, a wide-net loop closure at the orange start gate) builds the
map from wheel odometry, IMU heading and the coloured cones. Back at the start the map is
frozen, cone colours are repaired by which side of the car's own mapping-lap path they lie
on (the follower keeps that path near the middle, so it beats a camera mislabel), and the
track is built. With `feb_cone_ordering` in the stack that is the **team's cone ordering**
(below), asked once through its service: the rung midpoints are the centreline and the rung
lengths the width. Without it, or if it does not answer within 3 s, a built-in boundary walk
does the job (a step that follows the cone spacing and survives a missing cone, the width
re-measured, the mapping path filling in where the boundaries collapse). Then a
**minimum-curvature raceline** is solved as a sparse QP (CasADi, 10 ms for 1300 points) and
a **speed profile** laid over it from lateral, acceleration and braking limits. From then on
the car's speed comes from the wheel encoders put through the measured tyre curve (the throttle
sets the wheel speed at once, so the encoders on their own are the command coming back), its
position from dead reckoning corrected against the map by ICP on every lidar cone, colour-blind
and translation only (the IMU heading is absolute, and a rotation fitted to three cones in a row
would spin), and an **MPC** tracks the raceline: a kinematic bicycle with a steering-rate and a
tyre-force input, the formulation of the team's own controller, with the car's measured limits
and its 0.12 s actuation delay (direct multiple shooting, IPOPT, about 2 ms a solve). The
throttle is the planned tyre force through the inverse tyre curve, so the wheels are never spun
or locked. Without CasADi the node still maps and plans, and pure pursuit drives the raceline.
The vehicle model is `feb_cone_racer/vehicle.py`; `tools/dynid.py` measures it in the simulator
and `tools/dynfit.py` checks the model against the recording. `python3 -m feb_cone_racer.bench
<track.json>` drives the modelled car with the real controller, offline, in ten seconds. Debug topics: `/feb/cones`,
`/feb/map`, `/feb/raceline`, `/feb/pose`, `/feb/mpc_prediction`, and `/feb/status`, one line twice
a second with the phase (mapping lap, waiting for the ordering, racing), what is steering (local
path, reactive follower, MPC, pure pursuit) and any active fallback (map match lost, no cones in
view, stopped); `./feb-sim logs -f` shows the same story as the racer's log, and `debug: true` in
`config/racer.yaml` logs every lap-one steering decision and every scan that matched nothing.
Parameters are in `config/racer.yaml` and, for the racing half, `feb_cone_racer/params.py`. Cone colours: the camera is
192 by 108 pixels and a cone beyond 4 m is a few pixels, so a blob only colours a cone for the map
if it is brighter than the ground (a dark speck in the right hue used to hand a lidar cone the
wrong colour one time in ten); dim blobs still colour cones for driving, votes carry per tracked
cone, and a reading from far away counts for less. In testing (2026-09-29, every run with its own mapping lap) it laps
all seventeen cone tracks without touching a cone, each lap within 0.2 s of its plan: the loop in
7.5 s after a 36 s mapping lap, comp 2021 in 34.3 s, Spa in 84.4 s. `v_max` and `a_lat` in
`config/racer.yaml` are the knobs for more speed; the measured limits are in `vehicle.py`.

### Taking the racer to the real car: `calibrate_camera`

Everything in `config/racer.yaml` under *camera* is about this car's camera and has to be
fitted again for another one. Record a bag while driving past cones (lidar, `front_camera`,
`imu`; a minute or two, some corners) and run

```bash
ros2 run feb_cone_racer calibrate_camera <bag> --params config/racer.yaml
ros2 run feb_cone_racer calibrate_camera <bag> --params config/racer.yaml --track tracks/fsg/track.json   # simulator bag with IPS truth
```

It pairs lidar cones with the camera blobs that clearly belong to them (one blob at the cone's
expected column, nothing else on that bearing) and fits, in one least-squares problem, where a
point in front of the lidar lands in the image: `camera_hfov_deg`, `camera_ahead`,
`camera_lateral`, `camera_col_bias` and `camera_lag_s` (how much older than the scan the image
is, from the error against yaw rate). From the same blobs it proposes the image band the cones
live in (`band_top`, `band_bottom`, `band_floor`) and the brightness gate (`blob_min_value`).
With `--track`, the true cones and the bag's IPS say which colour every cone really is, so it
also fits the HSV bounds per colour from the cones' own pixels and reports the colour accuracy
by range. It prints a yaml snippet with the fit quality beside it (pairs used, residual in
pixels): under about 5 px is a good fit, and on the simulator's own bag it reproduces the
numbers in `racer.yaml`. What does not transfer by calibration: the vehicle model (`mass`,
tyres, the longitudinal fit) and the raceline margin, which are the car's, and the MPC weights.

### `feb_cone_ordering`: the team's cone ordering, as a node

`src/algorithms/` is a verbatim copy of the FSAE stack's `cone_ordering` (Akhil Agarwal,
feb-system-integration commit e048173), without its raylib visualiser; the node around it
speaks the simulator's messages. A Delaunay triangulation over all cones keeps the same-colour
edges under 10 m; a wall-following walk from the edge nearest the car extracts each colour's
boundary polygon; a parity test says whether the car sits inside a closed loop; every boundary
edge contributes a "force" rotated by 5/8 pi (blue one way, yellow the other) falling off with
the cube of the distance, and integrating that slope field from the car gives a path down the
middle of the corridor; every second path point is projected sideways onto both boundaries
(rotated when a rung would cross the previous one, snapped to the polygon), giving aligned
blue and yellow lists, one rung per 0.2 m along the track. It never chains cones, so a
missing cone only bends the field locally; a mislabelled one can still collapse a closed map's
ordering, so the node's wrapper flips the most suspicious cones and retries (no change to the
algorithm itself). On the recorded Spielberg map:
1693 rungs, widths 2.1 to 2.8 m, midpoints never closer than 1.0 m to a cone, no crossing
rungs, in well under a second.

It runs live (`ros2 launch feb_cone_ordering order.launch.py`: rungs on `/feb/cone_order/blue`
and `/feb/cone_order/yellow` for every `/feb/map`), as the `/feb/order_cones` service the
racer calls, or on a text file with `cone_order_cli` (see its README). The devkit builds the
C++ on start; the racer's launch starts the node when the package is present.

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
