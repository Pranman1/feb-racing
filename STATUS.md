# STATUS

Newest first.

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
