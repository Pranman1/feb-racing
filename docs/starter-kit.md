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
`feb-sim submit`. Your code stays on your laptop and is built inside the container on start.

## Stage 0: read the car

Topics (namespace `/autodrive/roboracer_1/`): `lidar` (1080 beams, 270°, 0.06 to 10 m, 40 Hz),
`imu`, `left_encoder` / `right_encoder` (wheel angle in rad, 0.059 m radius), `steering` and
`throttle` feedback. Commands: `steering_command` in [-1, 1] = ±0.5236 rad, `throttle_command`
in [-1, 1] where 0 is a hard brake. Steering lags about 0.25 s and slews at 3.2 rad/s.
`./feb-sim shell` gives you a ROS 2 shell: `ros2 topic hz`, `ros2 topic echo`, `rqt`.

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

## Stage 5: realism

`./feb-sim run --noise 1` adds lidar range noise and dropouts; `--noise "range_sigma:=0.05 latency:=0.05"`
sets them; `--lidar-hz 10` runs the lidar at the rate the real bridge often delivers.
`./feb-sim run --stack "..." --opponent normal` races you against the stock driver in car two
(`slow`, `normal` or `fast`), through the same proxy the competitions use.

## Submitting for competitions

```bash
docker build -t <you>/feb-racer:v1 stack/       # FROM ghcr.io/pranman1/feb-devkit, see starter_kit/Dockerfile
docker push <you>/feb-racer:v1
```
Add your team and image to `submissions.yaml` in a pull request. The image must start the
bridge and your stack from `/home/autodrive_devkit.sh` (the starter Dockerfile does).
