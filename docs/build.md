# Building the FEB Simulator

The simulator is the `feb-sim` repo: a fork of the AutoDRIVE Simulator (Unity 2022.3.52f1,
HDRP) with everything FEB-specific under `Assets/FEB`. The RoboRacer vehicle prefab
(`Assets/Prefabs/F1TENTH/F1TENTH.prefab`) and the upstream scripts are untouched.

## One-time setup on the build PC

1. Unity editor without root: `feb-racing/tools/unity_install.py <downloads> ~/Unity/Hub/Editor/2022.3.52f1`
   (already done on the team PC; see `docs/unity-licence.md` for activation).
2. Clone the fork: `git clone -b feb git@github.com:Pranman1/feb-sim.git` next to `feb-racing`.
   The shipped tracks are copied from `../feb-racing/tracks` at build time (`FEB_TRACKS` overrides).
3. Unzip the large upstream assets once: `Tools/unzip-and-clean.sh` (only needed to open the
   other AutoDRIVE scenes; the FEB scene does not use them).

## Build

```bash
cd feb-sim
./compile_check.sh      # compiles Assets/FEB against the editor assemblies, no licence needed
./build.sh scene        # regenerates Assets/Scenes/FEB Racing.unity from the upstream scene
./build.sh linux        # Builds/linux/FEB Simulator.x86_64   (organiser harness, Linux members)
./build.sh mac          # Builds/mac/FEB Simulator.app        (Apple Silicon + Intel)
./build.sh windows      # Builds/windows/FEB Simulator.exe
```

The first build imports the whole project (30 to 90 minutes); later builds take minutes.
Logs are in `Logs/`. Package the players for the release page:

```bash
tar -C Builds/linux -czf FEB-Simulator-linux.tar.gz .
(cd Builds/mac && zip -qr ../../FEB-Simulator-mac.zip "FEB Simulator.app")
(cd Builds/windows && zip -qr ../../FEB-Simulator-windows.zip .)
```

Upload the three archives to a GitHub release of `feb-racing`; `feb-sim setup` downloads
`.../releases/latest/download/<archive>`.

## What the fork adds (Assets/FEB)

| File | Role |
|---|---|
| `Scripts/TrackLoader.cs` | Builds walls, checkpoints, cones, spawn and cameras from a track folder at runtime; clones the car for `--cars N` |
| `Scripts/TrackData.cs`, `TrackLibrary.cs` | track.json model and where tracks are found (StreamingAssets/Tracks, `~/.feb-sim/tracks`) |
| `Scripts/FebLaunch.cs`, `FebAutoStart.cs` | `--track --connect --mode --lidar-hz --cars` command-line options |
| `Scripts/FebTrackMenu.cs` | the Track button in the menu |
| `Scripts/GhostLap.cs` | records and replays the best lap as a translucent car |
| `Editor/FebScene.cs` | generates the FEB Racing scene from the upstream RoboRacer Sim Racing scene |
| `Editor/FebBuild.cs` | player builds, icon, shipped tracks |

Command-line options of the built app (Unity's own `-batchmode -nographics -logFile` also work):

```
--track <name|folder>   --connect <host:port>   --mode manual|autonomous
--lidar-hz <rate>       --cars <n>
```
