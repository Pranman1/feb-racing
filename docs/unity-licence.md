# Activating Unity on the build PC

The FEB Simulator is built with Unity 2022.3.52f1, installed without root under
`~/Unity/Hub/Editor/2022.3.52f1`. Unity Personal is free for the team, but the editor
must be activated once per machine. Two ways:

## A. Unity Hub sign-in (easiest, needs the PC's screen, e.g. via Moonlight)

```bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority ~/unityhub/usr/lib/unityhub/unityhub-bin --no-sandbox &
```

1. Sign in (or create a Unity ID) in the Hub window.
2. Installs -> Locate -> pick `~/Unity/Hub/Editor/2022.3.52f1/Editor/Unity`.
3. Preferences -> Licenses -> Add -> "Get a free personal license".

## B. Manual activation (no GUI)

1. An activation request is already generated: `FEB/Unity_v2022.3.52f1.alf`
   (regenerate with `Unity -batchmode -nographics -quit -createManualActivationFile`).
2. Upload it at https://license.unity3d.com/manual and download the `.ulf` file.
3. Activate:
   ```bash
   ~/Unity/Hub/Editor/2022.3.52f1/Editor/Unity -batchmode -nographics -quit -manualLicenseFile ~/Downloads/Unity_v2022.x.ulf
   ```

Check: `~/Unity/Hub/Editor/2022.3.52f1/Editor/Unity -batchmode -nographics -quit -logFile - | grep -i licen`
should no longer say "No ULF license found".

Then build everything: `cd FEB/feb-sim && ./build.sh all`.
