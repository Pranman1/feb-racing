# STATUS

Newest first.

## 2026-09-16 - hardware options report, real circuits, two-car start fixed

- Report for the budget committee (superseded 2026-09-24 by `docs/proposal/overleaf/main.pdf`; the old file is deleted): 12 pages, the
  simulator platform with screenshots, six hardware plans with full bills of materials (compute,
  lidar, power, mounting, track kit), the Tamiya and 1:5-scale options analysed as infeasible,
  3D-printed FSAE cones, camera + lidar fusion with the owned RealSense, off-season case, one-on-one
  racing figures. Plan B (RoboRacer reference car, Hokuyo, track) recommended at ~$3,200.
- Tracks: `tools/track_from_centerline.py` imports the RoboRacer racetrack database
  (github.com/f1tenth/f1tenth_racetracks). `tracks/spielberg` is the Red Bull Ring at 1:10
  (342 m, 2.2 m corridor). `tracks/circuit` is a custom 138 m hairpin/esses/chicane layout.
  Both are `qualifying: false` (10 laps do not fit the 300 s attempt): shown on the site as
  showcase, excluded from qualification and nightly reruns. New `qualifying` key documented.
- Two-car start bug (found while taking race screenshots): both cars took a burst of hits in
  the first second. Cause: the car behind starts inside its own lidar bubble's view of the car
  ahead, steers into the wall, and the upstream respawn puts it at checkpoint 1, ahead of the
  spawn; the slower house racer then leads and the faster member car hits walls trying to pass in
  a 2.2 m corridor. Fixes: single-file grid 2.5 m apart (car one in front), and a `follow_gap`
  mode in the reactive driver (the house racer sits behind a car within 2 m instead of steering
  around it). Spielberg race: 0 hits for both cars over 70 s. On the 36 m loop the faster car
  laps the house racer within a minute and passing still produces contact; a lane-holding mode
  was tried and reverted (it scraped corners). Passing in a 2.2 m corridor is the racecraft
  problem the head-to-head final is meant to test; practice against the house racer is best on
  the circuit or Spielberg, or with `--opponent fast` (equal speed, no lapping).
- Starter kit on tight hairpins (the circuit) clips walls at 2.5 m/s; clean on the loop and
  Spielberg. Noted for the learning path (tune `lat_accel`, `max_speed` per track).
- Launcher: `FEB_DEVKIT_PORT` override.

## 2026-09-14 (night) - simulated time end to end: stamps, /clock, RTF, frame cap, camera cache

Context: a note from the RoboRacer session (verified against this fork) showed that under GPU
contention Unity advances physics by at most 0.1 s per frame, telemetry is frame-bound, the
payload had no time field, and the frame rate was uncapped. Drivers dividing encoder distance
by wall time then read speed 2-3x low during stalls and braked into corners.

All fixes are in the simulator and the devkit; competitors change nothing.
- Simulator: `FebRealTime` (Assets/FEB) tracks simulated time and a smoothed real-time factor
  (RTF). `Socket.cs` adds `Sim Time` and `Real Time Factor` to every telemetry message and
  reads each bridge camera back at most `--camera-hz` (10) times per simulated second, reusing
  the frame in between (the readback was the costly part of a message). `RenderingQuality.cs`
  caps rendering at `--fps` (60; 0 = uncapped) instead of -1. HUD shows `RTF x.xx`, red < 0.9.
  Maximum Allowed Timestep stays 0.1 s (documented).
- Devkit bridge (our copy): stamps every message with `Sim Time`, publishes `/clock`
  (rosgraph_msgs) and `/autodrive/roboracer_1/real_time_factor`. Starter launch files and the
  house racer run with `use_sim_time`. Starter driver never drops a scan silently (long gap is
  clamped, still commands).
- Proxy: `--rate` paces in simulated time when the payload carries `Sim Time`.
- Harness: the probe measures RTF from `/clock`; `result.json` records `real_time_factor`.
- Launcher: `--fps`, `--camera-hz`; `FEB_DEVKIT_PORT` env override (another bridge held 4567 on
  this PC during the test).
- Measured on this PC (with the roboracer session's headless league sim also running):
  idle: lidar 12 Hz, sim gap = wall gap 0.082 s, RTF 1.000. Under a full-screen xwd grab loop:
  wall gaps up to 0.285 s but simulated gaps at most 0.168 s, RTF 0.907 measured vs 0.92
  reported by the simulator. Headless starter practice unchanged: 11 laps, 0 hits, 14.04 s
  best, RTF 1.0 in the result.
- Not done, and why: emitting telemetry from a physics-side timer independent of rendering is
  not possible on Unity's main thread (FixedUpdate catch-up steps run inside a frame too); the
  cap plus the camera cache remove most of the frame cost instead, and the sim-time stamps make
  the remainder harmless to drivers.

## Future plans (not started)

- 1:5 vehicle model and RoboSense M1 sensor model in the simulator: the team owns an M1 and
  has an outdoor test track in Alameda; the hardware report costs a 1:5 car at ~$3,500 with
  them and defers it until the simulator can model it.
- RoboSense M1 on the nose: a solid-state scan pattern (120 x 25 deg, ~0.2 deg, 10 Hz) on the
  existing GPU lidar component, attached at runtime with no mass so the prefab and physics stay
  as they are. Needs a downsampled cloud or a binary side channel (a full 75k-point cloud is
  ~12 MB/s, beyond the JSON bridge), a tilt/crop for the 1:10 scale, and a 3D noise node.
  Training and the cone-track perception project only; never in a scored event (league sensor
  set is fixed). Estimate: 2-3 days for a downsampled version.
- Showcase camera: a slow orbit around the car for site and recruiting shots (no side profile
  exists today).
- Human-versus-human exhibition mode (two keyboards): car two is bridge-only by design.
- Wing wrap tuning: wider top-surface streaks, a pinstripe around the rear panel.

## 2026-09-14 (later) - ducts inside-out, app install folder, feb-sim update, HUD clock

- Ducts: the tube mesh was wound inside-out (near side culled, far inside visible: the
  "see-through" walls). Winding flipped; verified on screen.
- Install bug: the app archive unpacked into `~/.feb-sim/app/` while the launcher looks in
  `~/.feb-sim/app/<os>/`. A member's first `feb-sim setup` would have failed. Now unpacks into
  the per-OS folder (`unzip` on Mac to keep permissions); tested into a temporary folder.
- `feb-sim update`: git pull, docker pull, and the app is replaced when the release asset is
  newer than the installed one (stamp in `app/release.json`); `feb-sim run` prints one line
  when an update exists. The app also reads the repo's `tracks/` (`--tracks`), so a pulled
  track shows in the Track button without an app update.
- HUD clock: starts at the car's first crossing of the start line (`FebStartLine` on the
  finish trigger); the upstream LapTimer, which scoring reads, is unchanged.
- Packaging: exclude the stray `runs/` folder from the Linux archive.
- Branding: the Unity splash logo is now a "FEBAUTO Sim" card (was the AutoDRIVE logo; the
  "Made with Unity" part cannot be removed on a Personal licence) and the toolbar title reads
  "FEBAUTO Sim | <track>". App and archive file names stay "FEB Simulator".
- Wing wrap: the deck and the "Rear Shock Tower" (which is the wing, endplates and rear panel
  in this CAD) get planar texture coordinates on the renderer's own mesh copy and a generated
  texture: blue with gold edge streaks (endplates) and a gold trailing edge. The base plate and
  the crash members get the inverse wrap (gold, blue edge streaks). Colliders and the prefab
  asset untouched; verified on screen.
- HUD clock: the start-line crossing is detected geometrically (behind the line to ahead of
  it); the grid is so close to the line that the trigger fired at spawn. Verified: "--" on the
  grid.

## 2026-09-14 - Visual look, race HUD, menu icons, qualification rule

- Look: `FebLook.cs` dresses the scene at load (Visual, default): floor tint, an asphalt ribbon
  along the corridor (width from the nearest wall or cone) with white edge lines, white ducts
  with FEB blue bands (UVs added to the tube mesh), a chequered start line and gold checkpoint
  marks, a gradient sky and a warmer sun (runtime HDRP Volume, priority 10), and the livery:
  deck (wing + rear panel) FEB blue, chassis plate and crash members gold, rear decals on a
  white plate. All material instances and collider-free meshes: the prefab and physics are
  untouched. `--look simple` / the menu's Look button give the bare scene; remembered in
  PlayerPrefs. Headless runs force Simple.
- HUD: `FebHud.cs` replaces the small upstream lap panel with a bottom-centre panel: lap time
  large, last/best/lap/hits, speed. Same in both looks.
- Menu: Track, Cars and Look rows have their own icons (loop, two cars, sun).
- Verified on screen on Linux: loop (Driver's Eye), loop_cones (Trackcam), Porto, Simple.
- Qualification: `entries: qualified` events, `feb-race qualified`, Teams page ticks, `top: N`
  seeds for the bracket. Rule text in both manuals.

## 2026-09-14 - Foxglove instead of rviz; FEBAUTO Racing name; two manuals

- Visualisation: rviz on the laptop cannot see the container's topics (ROS_LOCALHOST_ONLY plus
  the Docker bridge network) and Mac/Windows members have no ROS at all. The devkit image now
  runs `foxglove_bridge` (port 8765, `FEB_FOXGLOVE=0` disables; the harness and the opponent
  container disable it). `feb-sim run` publishes the port and opens the Foxglove desktop app
  on it via its `foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765` deep link
  (`--no-foxglove` skips). Verified on the PC: 24 topics advertised, camera at 10 Hz in the app.
  Note: foxglove_bridge 3.4 speaks the `foxglove.sdk.v1` subprotocol; the current desktop app
  does too. Foxglove needs a one-time free sign-in.
- Renamed the programme to FEBAUTO Racing everywhere users see it (site, README, docs,
  manuals). The Unity scene file keeps its `FEB Racing.unity` name (internal; a rename means a
  full rebuild and re-release).
- Manuals: `docs/manual/competitor.pdf` and `organiser.pdf` from LaTeX (shared platform
  section), built with the texlive Docker image. They fold in every docs/*.md page.

## 2026-09-14 - ego car and camera are the same car; nothing drives without a stack

- Bug: cloning car one for the grid also cloned its viewing cameras (Driver's Eye etc.), so the
  screen followed car two while the mode button drove car one. Fix: clones keep only their
  sensor cameras (render-to-texture); viewing cameras and audio listener are disabled on clones.
  Verified on screen: with the starter kit on car one and car two idle, the Driver's Eye moves
  with car one.
- Confirmed behaviour: `./feb-sim run` with no `--stack` starts no driver and no opponent. Car
  one stood still for the whole check (same `ips` twice, 4 s apart); car two is not even on the
  bridge (the devkit publishes `roboracer_1` only) until `--opponent` attaches a driver to it.
  The earlier demos drove because I had copied the starter kit into `stack/`; that copy is gone.
- Players rebuilt (Linux, Mac, Windows) and the v0.1.0 release assets refreshed.

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
- Race at 10 Hz, starter 2.5 vs lidar house 2.0: starter wins by 12 s, one contact each (the
  overtake). Cone-only tracks get the map follower as opponent (a gap-follower has no walls).
  Players rebuilt with max_cars; release refreshed.
- House racer is now the smooth lidar driver at 1.5/2.0/2.5 m/s (8 laps, 0 wall contact on the
  loop at 10 Hz, lap spread 0.02 s); the map follower stays as `--opponent map`. Tracks carry
  `max_cars` (Porto = 1, ducts now 33 cm).

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

## 2026-09-24
- Proposal `docs/proposal/overleaf/main.tex` / `.pdf` is now the single budget document; the earlier drafts (`car-budget`, `hardware-options`, `rc-budget`, `figs/`) are deleted. It carries an itemised bill of materials with a shop link and the 24 Sep 2026 price on every line (Amazon checked line by line, used where cheaper), the same list in `docs/proposal/bom.csv`, a walk through the official build guide confirming the list covers the whole build, and a camera line (Logitech C920, or RealSense D435i if none is found in the lab). Laser cutting and 3D printing are excluded on purpose (student shop account). Re-priced after NVIDIA's July 2026 Jetson increase (Orin Nano Super $399). Corrected the controller: Flipsky Mini FSESC 6.7 is 4S-minimum and cannot run on 3S; the build lists the Flipsky Mini FSESC 4.20 (3S-13S) or the VESC 6 MkVI. Controller is the reference VESC 6 MkVI (Flipsky 4.20 noted as a $178 saving, not chosen). Chassis: the Fiesta VXL is sold out at dealers, so the list carries the in-stock Fiesta BL-2s (74154-4, $330, +$100 if a VXL is found). Camera bought new: RealSense D435IF from DigiKey, $376 list (the team no longer has last year's unit). Camera from the RealSense store at its $354 list price (its $455 checkout total is split into the tax and shipping lines); S2 from a US Amazon seller ($349). Totals split into parts, 10.25% sales tax on US-shipped items, and shipping+duty ($255: VESC UK, power board India, camera post, domestic post). Parts $1,951 now, $2,300 with the S2; all in $2,366 and $2,750; $3,199 official reference.
- `starter_kit/feb_cone_driver` (uncommitted, in progress): cone-track baseline driver, lidar+camera fusion, chain ordering, centreline, pure pursuit; completes laps but can stall on a cone; lidar-based stall recovery added, untested.
- Mac competitor manual made official: `docs/manual/competitor_mac.tex/.pdf` (was `competitor_rough`; draft note and the bare-bones driver section removed, "Keeping up to date" section added with `./feb-sim update` and how to copy a new starter package). Linked from the handbook and the manual README. Checked today: both repos public, devkit image pulls anonymously from ghcr, release v0.1.0 assets (17 Sep) are newer than the last simulator source change (16 Sep).
- Both competitor manuals gained a shared appendix (`docs/manual/starter_driver.tex`): the starter package file by file, and the driver's per-scan logic with a ray-cast diagram (bubble, widest gap, deepest and middle beams, target, corridor). Preamble now loads tikz and enumitem.
- General competitor manual: step-by-step Windows (WSL 2, Docker Desktop, Git Bash, Python, gh, Foxglove, GPU note) and Linux (Ubuntu packages, docker group, gh, Foxglove, Vulkan note) install sections, parallel to the Mac manual; points Mac users to it.
- Setup manuals split per system: `docs/manual/competitor_setup_{mac,windows,linux}.pdf`, same title page, logo and header as the full manual, shared body plus per-OS install file; `competitor_mac` removed. `competitor.pdf` stays the full manual.

## 2026-09-25
- `starter_kit/feb_cone_driver`: the baseline cone driver, one file. Lidar clusters + camera colour (calibrated camera pose, focal length, image lag, HSV bounds and image band, all parameters), colour memory, side override for cones beside the car, side-aware cone pairing (a blue cone pairs with the yellow cone to the right of its chain, which is what stopped the U-turns at hairpins), pure pursuit at ~1.1 m/s, lidar-scene stall recovery. 7 laps in 240 s on `loop_cones`, no hits, no stalls, trail inside the cones. Needs `--look visual` (camera thresholds).
- Simulator: cones are now real sizes (7 inch blue/yellow, 12 inch orange with two white bands, 40-segment mesh, flat top). `tools/track_build.py` adds four orange start cones at the finish line; `loop_cones` regenerated (28 blue, 43 yellow, 4 orange); new `tracks/spielberg_cones` (342 m, 292 cones, 2.5 m spacing, qualifying false). Linux player rebuilt; Mac/Windows and the release follow with the racer.
- Calibration bags recorded with a scratch ground-truth driver (not shipped). Camera colour accuracy on the new cones: 96% against ground truth after recalibration.
- `starter_kit/feb_cone_racer` (new): mapping lap with the baseline follower + GraphSLAM (FSAE formulation, weighted colour votes, wide-net loop closure at the orange gate), then boundaries, centreline, colour repair, minimum-curvature raceline (CasADi QP) and speed profile, then ICP localisation + nonlinear MPC (dynamic bicycle, servo lag in the model, substepped RK4, IPOPT, ~10 ms) with a local-follower fallback and stall/lost recovery. On `loop_cones`: map closed at 31 s (75 cones), 11 consecutive MPC laps at 22.9-23.8 s, 12 laps in 300 s, no hits (baseline driver: 31 s laps). Deps scipy+CasADi installed once into `stack/.pydeps` by `install_deps.sh`. Speed scale 0.6 for now.
- Cone packages made to work on any track, tested overnight on `spielberg_cones` (2.5 m cone spacing, a 1.45 m-radius hairpin) as well as `loop_cones` (1 m spacing). What broke and what fixed it: the boundary walk and the followers' cone chaining had fixed steps (3.5 m / 1.8 m) and now follow the cone spacing; the min-curvature QP was dense (11 minutes for 1300 points) and is now a sparse pentadiagonal solve (10 ms); the centreline smoothing cut 0.6 m into the hairpin and the width was never re-measured (now 5 samples and re-measured); cone colours are repaired by their side of the car's own mapping-lap path; the width is nearly constant, so where the boundaries collapse the mapping path fills in; localisation uses every lidar cone (coloured or not) and a translation search with the IMU heading finds the car again when lost; both followers back up when they stop with nothing to follow and hold the wheel for a second when the target vanishes; the mapping lap closes on the gate match. Final numbers: baseline driver 2 Spielberg laps in 12 minutes, no cone touched; racer maps Spielberg in 291 s then 6 MPC laps of 199-201 s, no cone touched; on loop_cones 14 laps of 22-23 s per 6-minute run, with 0 to 2 brushed cones per run (it backs off and carries on). Speed model corrected (long_b 3.2, 23 m/s per unit throttle as measured); `race_speed_scale` stays 0.6: at 0.7-0.8 the map match slips at speed (pose errors of metres right before each contact) and cones get touched. Capping corrections and gating the wide net made it worse; that is the open problem for faster laps.
- Mac and Windows players rebuilt with the real-size cones and the two cone tracks; release assets replaced (`feb-sim update` picks them up).
- `starter_kit/feb_cone_ordering`: the team's cone ordering (feb-system-integration `cone_ordering`, Akhil Agarwal, commit e048173) vendored verbatim under `src/algorithms/`, wrapped in a node that speaks the simulator's messages: `/feb/order_cones` service, live `/feb/cone_order/{blue,yellow}` topics from `/feb/map`, and a `cone_order_cli` for text files. The racer asks it once when its map closes (async, 3 s timeout, boundary walk as fallback) and builds the track from its rungs; the launch starts the node when the package is in the stack. On the recorded maps: Spielberg 1693 rungs, loop 180, midpoint clearance >= 1.0 m, raceline clearance 0.69 m (walk: 0.55). Live: the devkit builds the C++ in 11 s; the track is ready 48 ms (loop) / 83 ms (Spielberg) after the map closes; Spielberg 5 MPC laps of 199-205 s with no cone touched, loop 13 laps of 22.7-23.9 s with one brush, i.e. the same racing as with the walk (localisation, not the track, is the limit).
- Track picker and a track library. The app's Track button now opens a panel (groups across the top: FEB, FEB cones, FSAE, RoboRacer, F1, F1 cones; the group's tracks under it easiest first with length and a five-dot difficulty; built in code in the HUD's style, `--picker` opens it at launch for screenshots). `track.yaml` gained `category` and `difficulty` (computed from length, sharp corners and tightest radius by `track_build.py`, hand-set where the number was silly; `cones.from` takes a real cone layout as it is). New tools: `track_from_cones.py` (AMZ FSSIM yaml and EUFS csv, scaled to a 2.2 m corridor, cones kept where they stand). Library, 28 tracks (rr_skirk and esses changed after this count): feb: loop (1), circuit (4); feb_cones: loop_cones (1), circuit_cones (4); fsae: fs_small_track (2), fs_rectangle (2), fs_boa_constrictor (2), fs_bone (2), fs_peanut (3), fsi (3), fsg (3), fs_comp_2021 (5), fs_hairpins_increasing_difficulty (5); f1: spielberg (3), monza (3), interlagos (4), silverstone (4), spa (5); f1_cones: spielberg_cones (3), monza_cones (3), interlagos_cones (4), silverstone_cones (4), spa_cones (5); roboracer: porto (2), rr_skirk (2), rr_berlin (3), rr_vegas (5). Dropped: EUFS garden_light and its_a_mess (self-touching corridors), the Levine map (not a loop), Skirk (a room, the starter driver hits its walls). The site groups its practice cards the same way. Tested: the simple cone driver on all 17 cone tracks for 150 s each, no cone hit anywhere (short tracks lap, long ones cover 130-150 m; a few Formula Student layouts use the back-up recovery, rectangle four times, comp 2021 three); the walled starter driver on the 11 duct tracks headless for 120 s, no hits except Porto (4, its known narrow section) and Monza (2 at a chicane). The racer is validated on loop_cones and spielberg_cones only; the FSAE layouts are next for it.
- Racer lap one now runs the way the car does it: the growing map is published every scan, the team's cone ordering orders it live, and the car follows the local path its rungs give (pure pursuit on the first midpoint past 1.4 m; the reactive follower takes over when the rungs are stale, too short or the ordering package is absent). Mapping range 6 m so the map reaches further ahead. loop_cones: lap one 90% on the local path, map closed 32.6 s, 12 MPC laps of 22.5-24 s with two brushes; spielberg_cones: map closed 302 s, 5 MPC laps of 207-210 s, no cone touched. The basic cone driver stays reactive on purpose.
- Simulator v0.3.0 released: the picker, 28 tracks. A Unity batch build hung once for four hours right after loading the project (log stopped at "Unloading unused serialized files"); killing it and rerunning worked, so watch the log when building.
- NEXT (agreed, not started): the simulator publishes a small "track changed" signal and the racer wipes its map and starts a fresh mapping lap on it, so the picker is the only control needed with the racer running. Half a day with a test.
- NEXT (agreed, not started): a camera calibration tool for the real car, next to sysid_fit: from one bag of a slow lap past known cones it fits the camera model (field of view, mounting offsets, image lag) and the HSV colour thresholds and band, and writes them into the cone packages' yaml, so a handover to the physical RC car is sysid bag + calibration bag + copy numbers.

## 2026-09-26
- Simulator: cones are light rigid bodies on a flat plate (0.3 kg, 0.6 kg for the big orange ones) and fall over or get shoved when hit, like real ones; a cone hit counts on the HUD (once per cone per two seconds, no respawn), which it never did before; the asphalt is painted between the corridor's own edges (`edges` in track.json, so the paint follows real Formula Student layouts); no gold checkpoint marks on cone tracks (a camera reads them as yellow cones); the app keeps retrying the devkit connection for ten minutes (the first C++ build on a laptop takes that long); `--picker`.
- Racer: reversing removed by default (`recovery: stop`, the real car stops; `reverse` is an opt-in convenience). Root causes found this morning: a rejected negative wheel speed while backing up put a 4 m break into every map that included a reverse; the team's ordering collapses on a closed map with one mislabelled cone (wrapper flips suspects and retries; the racer accepts only a closed loop of rungs covering 70% of the lap, and falls back to the walk if the track pinches below 0.5 m); "no cones in view" counted camera cones only; the raceline ignored individual cones. Loop closure now also comes from a unique placement search of the cones in view against the first cones mapped, so the orange gate is no longer required. Lap one uses the lidar's widest opening when the known path ends at an unseen corner and slows when the path is short. `feb-sim logs`, `/feb/status`.
- Formula Student layouts: direction from the cone colours (five ran clockwise), corridor rasterised from the real boundaries.
