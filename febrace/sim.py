"""Where the FEB Simulator app lives on this machine."""
import os
import pathlib
import platform

APP_DIR = pathlib.Path.home() / ".feb-sim" / "app"
PLATFORM_DIRS = {"Linux": "linux", "Darwin": "mac", "Windows": "windows"}     # one sub-folder per OS under APP_DIR
EXECUTABLES = {
    "Linux": pathlib.Path("linux", "FEB Simulator.x86_64"),
    "Darwin": pathlib.Path("mac", "FEB Simulator.app", "Contents", "MacOS", "FEB Simulator"),
    "Windows": pathlib.Path("windows", "FEB Simulator.exe"),
}
ARCHIVES = {"Linux": "FEB-Simulator-linux.tar.gz", "Darwin": "FEB-Simulator-mac.zip", "Windows": "FEB-Simulator-windows.zip"}


def executable():
    """Path of the simulator executable for this OS (FEB_SIM_APP overrides), or None."""
    env = os.environ.get("FEB_SIM_APP")
    if env:
        return pathlib.Path(env)
    exe = EXECUTABLES.get(platform.system())
    return APP_DIR / exe if exe and (APP_DIR / exe).exists() else None


def platform_dir():
    """Where this OS's archive unpacks to (the archives hold the app at their root)."""
    return APP_DIR / PLATFORM_DIRS[platform.system()]


def archive_name():
    return ARCHIVES[platform.system()]
