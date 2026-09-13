"""Where the FEB Simulator app lives on this machine."""
import os
import pathlib
import platform

APP_DIR = pathlib.Path.home() / ".feb-sim" / "app"
EXECUTABLES = {
    "Linux": APP_DIR / "linux" / "FEB Simulator.x86_64",
    "Darwin": APP_DIR / "mac" / "FEB Simulator.app" / "Contents" / "MacOS" / "FEB Simulator",
    "Windows": APP_DIR / "windows" / "FEB Simulator.exe",
}
ARCHIVES = {"Linux": "FEB-Simulator-linux.tar.gz", "Darwin": "FEB-Simulator-mac.zip", "Windows": "FEB-Simulator-windows.zip"}


def executable():
    """Path of the simulator executable for this OS (FEB_SIM_APP overrides), or None."""
    env = os.environ.get("FEB_SIM_APP")
    if env:
        return pathlib.Path(env)
    exe = EXECUTABLES.get(platform.system())
    return exe if exe and exe.exists() else None


def archive_name():
    return ARCHIVES[platform.system()]
