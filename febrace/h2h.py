"""Head-to-head races: N cars in one simulator, one racing container per car.

A small proxy (febrace/proxy.py, run inside the devkit image) sits between the
simulator and the containers: every container still sees its own car as roboracer_1
while the simulator runs V1..VN. Each container is probed separately, so every car gets
a normal result record plus its finishing position. Positions: most laps first, then
finishing time including collision penalties.

The official Race Control Tower does the same id rewriting but freezes both cars after
any contact until a race director rules in its web UI; use it for stewarded live events,
not for automated brackets.
"""
import datetime as dt
import json
import pathlib
import os
import subprocess
import threading
import time

from . import results
from .runner import Attempt

PROXY = pathlib.Path(__file__).with_name("proxy.py")
PROXY_IMAGE = os.environ.get("FEB_DEVKIT_IMAGE", "ghcr.io/pranman1/feb-devkit:latest")


class Race:
    def __init__(self, entries, track, sim_exe, event, race_id, rules, base_port=4568, rct_port=4570,
                 headless=True, runs_dir=pathlib.Path("runs")):
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.cars = [Attempt(image=image, track=track, sim_exe=sim_exe, team=team, event=event, rules=rules,
                             port=base_port + i, headless=headless, runs_dir=runs_dir)
                     for i, (team, image) in enumerate(entries)]
        for i, car in enumerate(self.cars):
            car.id = f"{stamp}-{race_id}-{car.team.replace(' ', '_')}"
            car.dir = pathlib.Path(runs_dir) / f"{stamp}-{race_id}" / f"car{i + 1}"
            car.container = f"feb-h2h-{stamp}-{i + 1}"
        self.race_id, self.proxy_port, self.proxy = race_id, rct_port, f"feb-proxy-{stamp}"

    def run(self, on_event=print):
        started = dt.datetime.now(dt.timezone.utc)
        events = [[] for _ in self.cars]
        sim = None
        try:
            for car in self.cars:
                car.dir.mkdir(parents=True, exist_ok=True)
                car._start_container()
                car._wait_port()
            self._start_proxy()
            for car in self.cars:
                car._exec_detached("ros2 bag record -o /tmp/feb_bag /autodrive/roboracer_1/lidar /autodrive/roboracer_1/ips "
                                   "/autodrive/roboracer_1/steering_command /autodrive/roboracer_1/throttle_command "
                                   "/autodrive/roboracer_1/lap_count /autodrive/roboracer_1/collision_count")
            lead = self.cars[0]
            sim = subprocess.Popen([str(lead.sim_exe), "--track", str(lead.track), "--connect", f"127.0.0.1:{self.proxy_port}",
                                    "--mode", "autonomous", "--cars", str(len(self.cars)),
                                    "-logFile", str(lead.dir.parent / "sim.log")] + (["-batchmode", "-nographics"] if lead.headless else []))
            threads = [threading.Thread(target=self._watch, args=(car, events[i], on_event)) for i, car in enumerate(self.cars)]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
        finally:
            if sim is not None:
                sim.terminate()
            with open(self.cars[0].dir.parent / "proxy.log", "w") as log:
                subprocess.run(["docker", "logs", self.proxy], stdout=log, stderr=subprocess.STDOUT)
            subprocess.run(["docker", "rm", "-f", self.proxy], capture_output=True)
            for car in self.cars:
                car._collect_and_stop()
        return self._results(events, started)

    def _start_proxy(self):
        ports = [str(car.port) for car in self.cars]
        subprocess.run(["docker", "run", "-d", "--name", self.proxy, "--network=host",
                        "-v", f"{PROXY}:/tmp/proxy.py:ro", "--entrypoint", "python3", PROXY_IMAGE,
                        "-u", "/tmp/proxy.py", "--port", str(self.proxy_port), "--devkits", *ports], check=True, stdout=subprocess.DEVNULL)
        time.sleep(3)

    def _watch(self, car, sink, on_event):
        for ev in car._probe():
            ev["car"] = car.team
            sink.append(ev)
            on_event(json.dumps(ev))

    def _results(self, events, started):
        rows = []
        for car, evs in zip(self.cars, events):
            r = results.build(car, evs, started)
            r["mode"] = "head-to-head"
            r["race"] = self.race_id
            r["opponents"] = [c.team for c in self.cars if c is not car]
            finish = next((e["t"] for e in reversed(evs) if e.get("event") == "lap"), None)
            r["finish_s"] = round(finish + r["penalty_s"], 3) if finish is not None else None
            rows.append(r)
        rows.sort(key=lambda r: (-r["laps_completed"], r["finish_s"] if r["finish_s"] is not None else float("inf")))
        for position, r in enumerate(rows, 1):
            r["position"] = position
            (car_dir(self, r) / "result.json").write_text(json.dumps(r, indent=1))
        return rows


def car_dir(race, result):
    return next(c.dir for c in race.cars if c.team == result["team"])


def bracket(seeded_teams):
    """Single-elimination pairings for round 1 from a seeding list (1 v N, 2 v N-1, ...).
    Byes when the count is odd: the top seed skips the round."""
    teams = list(seeded_teams)
    byes = teams[:len(teams) % 2]
    rest = teams[len(byes):]
    return [(rest[i], rest[-1 - i]) for i in range(len(rest) // 2)], byes
