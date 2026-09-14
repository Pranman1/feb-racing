# STATUS

Newest first.

## 2026-09-14 (early) - bridge rate is the root cause; fixed 10 Hz scoring; drivers rate-robust

- Finding: with a window open the simulator exchanges data at ~10 Hz on Linux (camera readback
  per car per message); headless it is 20-40 Hz. Every driver had only ever been tested at 40 Hz.
  At 10 Hz the old house driver limit-cycled (overspeed brake -> stop -> restart) and the old
  starter driver weaved full lock and grazed walls (~0.6 collisions/s).
- Design: scoring is now at a fixed 10 Hz for everyone. The proxy has a time-based `--rate`;
  every scored run (single car too) goes through it at 10 Hz, matching what a laptop with a
  window gets. Drivers must be rate-agnostic; docs say so.
- House driver: no brake regime, event-driven (one step per pose), windowed speed estimate,
  lookahead grows with data latency, bounded pursuit curvature, follows a car ahead. 10 Hz: clean
  at 2.5 and 3.5 m/s (14.1 s / 10.9 s loop laps). 5 Hz: degrades (out of spec).
- Starter driver: ported the tuned reference (deep-weighted gap target, percentile clearance,
  slow-steering speed law, accel/decel-limited target, feed-forward + bounded trim, slew) plus
  actuator-delay compensation of the target bearing. 10 Hz: 14.1 s laps, one wall touch in four.
- Launcher: car one is autonomous by default whenever a stack is given (it defaulted to Manual,
  which is why "car one does not move" kept coming back); `--opponent` needs `--stack`.
- Two-car race at 10 Hz (starter vs house): both finish; 4-6 contacts between the cars over
  four laps as they run nose-to-tail. Counted for both per the rules.

## 2026-09-13 (late) - bug pass from the on-screen review

- Bridge: a devkit that drives fewer cars than the scene has no longer breaks the simulator
  (missing per-car command fields default to "no command"); extra cars sit still until a
  bridge drives them, and only car one takes the keyboard.
- Grid: two columns 0.7 m apart, rows 1 m apart; cars no longer touch at the start.
- Menu: upstream button style kept; "Track: <name>" and "Cars: <n>" rows cycle (Scene Light row
  removed). Driver's Eye is the default view.
- Decals: FEB mark on the rear panel's left, "FEB AUTO" on its right.
- House driver rewritten (`devkit/feb_tools/feb_tools/house_driver.py`): centreline pursuit with
  curvature feed-forward capped at the slip peak, speed profile with braking pass, feed-forward
  throttle with bounded trim (no accelerate/brake limit cycle). Loop 14.4 s laps at 2.5 m/s,
  10.8 s at 3.5; Porto 12.3 s / 10.0 s; zero wall contact, lap-to-lap spread 0.05 s.
- `feb-sim run --opponent` needs `--stack` (no opponent without your own car driving).
- Harness proxy moved to port 4580 (three cars collided with it on 4570). Verified headless:
  two-car race clean (0 collisions); three-car race completes laps but the house drivers do not
  avoid each other, so 3+ cars stays experimental.
- Porto: walls are now built by offsetting each boundary along its normals, so the thin island
  is one loop (dilation used to split it into two).
- Starter driver: same throttle law as the house driver (feed-forward + bounded trim, low-passed
  steering/clearance/target, slew limit): five clean laps headless, throttle never drops to zero.
- Dropped the unused LLMUnity plugin from the fork: it injected 780 MB of native libraries into
  every player. Players are now 296 MB (Linux), 261 MB (Mac), 360 MB (Windows); release refreshed.

## 2026-09-13 (night) - seen on screen, opponent mode, handbook

- After the reboot the simulator runs on the PC's GPU. Fixed from what we saw: cones (upstream
  cone is a SketchUp asset with no mesh on Linux; cones are now generated in code, striped),
  manual mode drives car one only, Track and Cars dropdowns replace the Scene Light row, FEB rear
  decal, `--camera` option, duct colour per track (`walls.color`).
- `feb-sim run --opponent slow|normal|fast` races the stock starter driver in car two through the
  proxy; verified on screen with both cars lapping.
- `docs/handbook.md` is the single guide for competitors and the organiser.
- Mac/Windows rebuilt with these changes; release v0.1.0 assets refreshed.

## 2026-09-13 (evening) - simulator built and racing; repos and site live

- Unity licensed (Hub sign-in), `feb-sim` scene generated and Linux + Mac players built.
  Windows build first failed on HDRP's DLSS code (NVIDIA module); retrying without the module.
- End to end on this PC: `feb-race run` with the starter-kit image on `tracks/loop` -> the car
  drives, laps count on the generated checkpoints, wall hits count, lap times match the sim's
  timer, a result JSON and a bag are written. Fixed on the way: spawn pose must be applied to the
  rigidbody too; the probe must wait for last_lap_time to change (it lags lap_count by a message).
- Repos pushed: github.com/Pranman1/feb-racing (main) and github.com/Pranman1/feb-sim (feb).
  Site live at https://pranman1.github.io/feb-racing/. Devkit image built by Actions
  (ghcr.io/pranman1/feb-devkit, still private until flipped in the package settings).
- Two-car head-to-head verified: two starter-driver containers, our proxy (`febrace/proxy.py`),
  simulator with `--cars 2`; both cars finished warm-up + 2 laps with no contact, positions by
  finish time. Two lessons: the official Race Control Tower freezes both cars after any contact
  until a human rules in its UI (kept as an option for stewarded finals, not used by the harness),
  and containers on one Docker network discover each other's ROS topics, so every container now
  runs with ROS_LOCALHOST_ONLY=1.
- Windows player builds after dropping the NVIDIA (DLSS) module. All three players rebuilt from
  the final code and published: https://github.com/Pranman1/feb-racing/releases/tag/v0.1.0
  (`feb-sim setup` downloads from there).
- Still needs a reboot of the PC: any windowed simulator run (NVIDIA driver/library mismatch:
  `nvidia-smi` fails, GLX context creation fails). Headless runs are unaffected. After the reboot,
  run `./feb-sim run --track loop_cones --cars 2` once to eyeball walls, cones, ghost and the menu.

## 2026-09-13 - M1 to M4 code complete; builds blocked on the Unity licence

**Done and verified on this PC**
- Track pipeline (`tools/track_design.py`, `tools/track_build.py`): tested on a designed loop
  and the RoboRacer Porto SLAM map; both in `tracks/` with previews.
- Devkit image `devkit/` (ROS 2 Humble + AutoDRIVE bridge + `feb_tools` noise node): builds,
  connects to the official simulator binary, publishes the 20 league topics; noise/dropout and
  the starter driver verified with synthetic scans inside the container.
- Starter kit `starter_kit/feb_driver`: builds in the container, drives from lidar, sysid probe + fit tool.
- Harness `feb-race` / `febrace/`: attempt runner (container + headless sim + in-container probe +
  bag + audit of restricted topics), rules (league scoring), results schema + validation, site
  generator, head-to-head races through the official Race Control Tower (image `feb-rct` built)
  and knockout brackets. Plumbing verified with a dummy simulator; real runs need the build.
- Site `site/index.html` + `.github/workflows/` (Pages deploy, multi-arch devkit image to GHCR,
  auto-merge of members' practice results).
- Unity fork `../feb-sim` branch `feb`: `Assets/FEB` (runtime track loader with walls/cones/
  checkpoints/spawn/cameras, N cars, CLI options, auto-connect, track menu, ghost lap, scene
  generator, build scripts, FEB branding). `compile_check.sh` passes against the editor assemblies.
- Unity 2022.3.52f1 + Mac/Windows/Linux modules installed under `~/Unity` without root.

**Blocked on you (in this order)**
1. Unity licence: `docs/unity-licence.md` (Hub sign-in on the PC screen, or upload
   `FEB/Unity_v2022.3.52f1.alf` at license.unity3d.com/manual). Then `cd ../feb-sim && ./build.sh all`
   and test: `FEB_SIM_APP=../feb-sim/Builds/linux/"FEB Simulator.x86_64" ./feb-sim run` and
   `./feb-race run --image feb-devkit:dev --team "Example Team" --track tracks/loop --gui`.
2. GitHub: `gh auth login`, then
   `gh repo create feb-racing --public --source . --push` (in feb-racing) and
   `gh repo fork AutoDRIVE-Ecosystem/AutoDRIVE --clone=false --fork-name feb-sim`,
   `git -C ../feb-sim remote add fork git@github.com:Pranman1/feb-sim.git && git -C ../feb-sim push -u fork feb`.
   Enable Pages (Settings -> Pages -> Source: GitHub Actions). The devkit image then builds to
   `ghcr.io/pranman1/feb-devkit` (make the package public once).
3. Upload the three player archives to a GitHub release of feb-racing (`docs/build.md`).

**Not yet exercised end to end** (needs the build): lap counting on generated checkpoints, the
ghost, cones, the track menu, 2-car spawning with the RCT. All compile; expect small fixes.
