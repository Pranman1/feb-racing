"""Run one attempt: start the racing container, start the simulator against it, watch
the lap and collision telemetry from inside the container, record a bag, tear down.

Used by the organiser's `feb-race` (headless Linux build, submitted images) and by the
members' `feb-sim practice` (their window, their devkit container with ./stack mounted).
"""
import datetime as dt
import json
import pathlib
import shutil
import socket
import subprocess
import time

from . import results, rules as rules_mod

ROS_ENV = "source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash"
PROBE = pathlib.Path(__file__).with_name("probe.py")
BAG_TOPICS = ["lidar", "imu", "left_encoder", "right_encoder", "steering", "throttle", "steering_command",
              "throttle_command", "ips", "lap_count", "collision_count", "last_lap_time"]


class Attempt:
    def __init__(self, image, track, sim_exe, team="unknown", event="practice", rules=None, port=4568,
                 headless=True, stack_dir=None, stack_cmd="", runs_dir=pathlib.Path("runs")):
        self.image, self.track, self.sim_exe = image, pathlib.Path(track).resolve(), sim_exe
        self.team, self.event, self.rules = team, event, rules or rules_mod.Rules()
        self.port, self.headless, self.stack_dir, self.stack_cmd = port, headless, stack_dir, stack_cmd
        self.id = dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + team.replace(" ", "_")
        self.dir = pathlib.Path(runs_dir) / self.id
        self.container = "feb-run-" + self.id

    # ------------------------------------------------------------ lifecycle

    def run(self, on_event=print):
        self.dir.mkdir(parents=True, exist_ok=True)
        started = dt.datetime.now(dt.timezone.utc)
        sim = None
        events = []
        try:
            self._start_container()
            self._wait_port()
            self._exec_detached(f"ros2 bag record -o /tmp/feb_bag " + " ".join("/autodrive/roboracer_1/" + t for t in BAG_TOPICS))
            sim = self._start_sim()
            for ev in self._probe():
                events.append(ev)
                on_event(json.dumps(ev))
        finally:
            if sim is not None:
                sim.terminate()
            self._collect_and_stop()
        result = results.build(self, events, started)
        (self.dir / "result.json").write_text(json.dumps(result, indent=1))
        return result

    def _start_container(self):
        cmd = ["docker", "run", "-d", "--rm", "--name", self.container, "-p", f"{self.port}:4567"]
        if self.stack_dir:
            cmd += ["-v", f"{pathlib.Path(self.stack_dir).resolve()}:/home/autodrive_devkit/src/stack",
                    "-e", f"FEB_LAUNCH={self.stack_cmd}"]
        subprocess.run(cmd + [self.image], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(["docker", "cp", str(PROBE), f"{self.container}:/tmp/feb_probe.py"], check=True)

    def _wait_port(self, timeout=120):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with socket.socket() as s:
                s.settimeout(0.5)
                if s.connect_ex(("127.0.0.1", self.port)) == 0:
                    return
            if not subprocess.run(["docker", "ps", "-q", "-f", f"name=^{self.container}$"], capture_output=True, text=True).stdout.strip():
                raise RuntimeError("container exited before the bridge came up")
            time.sleep(1)
        raise RuntimeError(f"bridge did not open port {self.port} within {timeout} s")

    def _start_sim(self):
        cmd = [str(self.sim_exe), "--track", str(self.track), "--connect", f"127.0.0.1:{self.port}",
               "--mode", "autonomous", "-logFile", str(self.dir / "sim.log")]
        if self.headless:
            cmd += ["-batchmode", "-nographics"]
        return subprocess.Popen(cmd)

    def _probe(self):
        cmd = f"python3 /tmp/feb_probe.py --laps {self.rules.total_laps} --timeout {self.rules.timeout_s}"
        proc = subprocess.Popen(["docker", "exec", self.container, "bash", "-c", f"{ROS_ENV} && {cmd}"],
                                stdout=subprocess.PIPE, text=True)
        for line in proc.stdout:
            line = line.strip()
            if line.startswith("{"):
                yield json.loads(line)
        proc.wait()

    def _exec_detached(self, cmd):
        subprocess.run(["docker", "exec", "-d", self.container, "bash", "-c", f"{ROS_ENV} && {cmd}"], check=True)

    def _collect_and_stop(self):
        subprocess.run(["docker", "exec", self.container, "pkill", "-INT", "-f", "ros2 bag record"], capture_output=True)
        time.sleep(2)
        subprocess.run(["docker", "cp", f"{self.container}:/tmp/feb_bag", str(self.dir / "bag")], capture_output=True)
        with open(self.dir / "container.log", "w") as log:
            subprocess.run(["docker", "logs", self.container], stdout=log, stderr=subprocess.STDOUT)
        subprocess.run(["docker", "rm", "-f", self.container], capture_output=True)

    def image_digest(self):
        out = subprocess.run(["docker", "image", "inspect", "--format", "{{index .RepoDigests 0}} {{.Id}}", self.image],
                             capture_output=True, text=True).stdout.split()
        return out[0] if out and "@" in out[0] else (out[-1] if out else "")
