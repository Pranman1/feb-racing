# Plan: system identification for the real car

Status: planned 2026-10-02, no code yet. Lives in `starter_kit/feb_cone_racer/tools/sysid/`,
next to today's `dynid.py` and `dynfit.py`, which stay as they are until this replaces them.

## The problem

The racer's vehicle model is 18 constants in `feb_cone_racer/vehicle.py`: wheelbase, max steer,
servo rate, actuation delay, wheel speed per throttle, drag, a tyre curve (stiffness, peak slip,
peak grip, sliding slip, sliding grip), lateral grip limits, cornering drag. They were measured
in the simulator with `tools/dynid.py`, which drives a fixed schedule of manoeuvres and records
the simulator's true body velocity. The real car has no true body velocity, no reset-to-start,
no steering feedback, and its wheel speed is the VESC's motor RPM. So the measuring tool has to
be rebuilt; the model, the manoeuvres and the fitting ideas carry over.

## What is the same on the car

- The model structure. The RoboRacer stack drives the VESC in speed mode (an ERPM target the
  VESC's own loop holds), so "the command sets the wheel speed and the tyre pulls the body after
  it" holds on the car as it does in the simulator; only the numbers differ.
- The manoeuvre schedule of `dynid.py`: throttle steps, lifts, zero-throttle brakes, launches at
  four levels, a ramp, a PRBS; steering steps at three speeds. It becomes a schedule of wheel
  speeds in m/s, converted per platform (throttle fraction in the simulator, ERPM on the car).
- The fitting: slip against acceleration for the tyre curve, coast for drag, yaw rate against
  steering for the bicycle, step response for the servo and the delay.

## What is different, and the answer to each

| On the car | Answer |
|---|---|
| No true body speed | Estimate it three ways and cross-check: scan-to-scan matching of the lidar's cone clusters (the racer's own `perception.clusters`, cones every metre round the test strip; a few cm/s), IMU acceleration integrated over 2 to 3 s bursts with the bias taken while stationary, and coast-downs where the freewheeling wheel *is* the body speed |
| Wheel speed is motor ERPM | Calibrate once: a measured roll of 20 m at walking pace gives metres per ERPM (gear ratio, pole pairs and tyre radius in one number) |
| No steering feedback | The servo is open loop. Identify its rate and delay from the lateral runs (yaw-rate step response to a steering step), then the racer runs `vehicle.py`'s servo model forward from the command |
| No reset between runs | The recorder waits for a button (the gamepad's dead-man) to start each run; a person walks the car back |
| Simulator clock | Wall clock |
| Nothing stops a bad command | A deadman through the stack's own mux, a speed cap per session, and an abort when the scan shows anything closer than 1.5 m ahead or the estimated position leaves the box |
| Zero throttle locks the wheels in the simulator | Speed mode 0 on the VESC brakes too. Coast-downs use duty 0, which frees the wheels |

## Files

```
tools/sysid/
  record.py    the manoeuvre runner and recorder. --platform sim|car picks the topic names and the
               command conversion; the schedule is shared. Records every tick: wheel speed,
               commands, IMU, the lidar's cone clusters, and the truth when the platform has it
               (unused by the estimators, kept for checking). One pickle per session.
  speed.py     body speed from a recording, three estimators (cones, IMU bursts, coast) and a
               comparison against the truth when present.
  fit.py       the 18 constants from a recording, with residuals, and the % error against
               vehicle.py when the truth is present. Writes vehicle.yaml.
  README.md    the procedure: the strip, the cone ring, batteries, order of runs, what to look at.
```

`vehicle.py` gains `load(path)`, which overrides its constants from a yaml, and the racer node a
`vehicle_file` parameter, so a fitted car is a file, not an edit.

## How it is proven before the car exists

Run `record.py --platform sim` in the simulator: the truth is recorded but the estimators never
see it. `fit.py` then reports every constant next to the known value. Acceptance: drag and peak
grip within 5%, the rest within 10%. Until that passes, nothing goes near the car. When it does,
the car session is data collection, not debugging.

## The car session

Flat tarmac, 30 m straight, cones every metre down both sides (they are the landmarks the speed
estimate needs, and the camera calibration bag comes from the same lap). Three batteries. Order:
1. Encoder calibration: two slow 20 m rolls against a tape measure.
2. Coast-downs from 2, 3, 4 m/s: drag, and a check of the encoder number.
3. Longitudinal schedule at the session's speed cap (2 m/s the first time, then 4, then 6).
4. Lateral schedule: steering steps at each speed, on the wide part of the lot.
5. One slow lap past the cones for `calibrate_camera`.

Two or three sessions, since the first one always finds something about the car itself.

## Effort

| Piece | |
|---|---|
| `record.py` from `dynid.py`, with platforms, no reset, a deadman | half a day |
| `speed.py`, validated against the truth in the simulator | half a day |
| `fit.py` | a day |
| `vehicle.load`, README, the simulator acceptance run | half a day |

About three days of code, then the sessions.

## Risks

- The VESC's speed loop has its own dynamics (a PID, a current limit) that the simulator's instant
  wheel speed does not. The step responses will show it; if it matters, the model gets a first-order
  lag on the wheel speed, one more constant.
- The VESC's on-board IMU may be too noisy for the burst integration; the cone-based estimate is
  the primary one, the IMU a check.
- 4WD: motor ERPM is one number for four wheels, and under hard launches the front and rear slip
  differently. The tyre curve fitted is the car's average, which is what the controller needs.
