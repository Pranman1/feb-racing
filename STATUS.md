# STATUS

Newest first. One entry per working session.

## 2026-09-13 - M1 in progress

- Track pipeline done and tested: `tools/track_design.py` (waypoints -> map.png) and
  `tools/track_build.py` (map.png -> track.json + preview.png). Works on a designed loop
  and on the legacy Porto SLAM map from the RoboRacer racetracks repo.
- Unity 2022.3.52f1 editor + Mac/Windows/Linux modules downloading to ~/Unity/dl (no root needed).
  Unity Hub extracted to ~/unityhub (runs on DISPLAY=:0, needed once for the licence sign-in).
- AutoDRIVE Simulator branch shallow-cloned to ../feb-sim (4.7 GB).
- Blocked on the user: Unity licence sign-in, `gh auth login`, Docker Hub login.
