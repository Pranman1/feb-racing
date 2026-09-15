"""Result records: one JSON file per attempt, the unit both the site and the leaderboard consume.

results/
    practice/<track>/<team>-<id>.json     self-reported by members (verified: false until rerun)
    events/<event>/<team>-<id>.json       written by the organiser's harness (verified: true)
"""
import datetime as dt
import json
import pathlib
import socket

from . import rules as rules_mod

SCHEMA = 1
ROOT = pathlib.Path(__file__).resolve().parent.parent / "results"


def build(attempt, events, started):
    end = next((e for e in events if e.get("event") == "end"), {})
    audit = next((e for e in events if e.get("event") == "audit"), {})
    lap_times = end.get("lap_times") or []
    collisions = end.get("collisions") or 0
    status, timed, penalty, total, best = rules_mod.score(attempt.rules, lap_times, collisions)
    if end.get("error"):
        status = "error"
    return {
        "schema": SCHEMA,
        "id": attempt.id,
        "event": attempt.event,
        "team": attempt.team,
        "image": attempt.image,
        "image_digest": attempt.image_digest(),
        "track": attempt.track.name,
        "mode": "time-attack",
        "rules": attempt.rules.to_dict(),
        "started_at": started.isoformat(timespec="seconds"),
        "duration_s": end.get("t"),
        "status": status,
        "laps_completed": end.get("laps") or 0,
        "real_time_factor": end.get("real_time_factor"),   # simulated / wall time over the run; ~1.0 is healthy
        "warmup_s": lap_times[0] if lap_times else None,
        "lap_times": [round(t, 3) for t in timed],
        "collisions": collisions,
        "collision_times": [e["t"] for e in events if e.get("event") == "collision"],
        "penalty_s": penalty,
        "total_s": total,
        "best_lap_s": round(best, 3) if best is not None else None,
        "restricted_subscribers": audit.get("restricted", {}),
        "verified": attempt.event != "practice",
        "runner": socket.gethostname(),
        "error": end.get("error"),
    }


def path_for(result):
    kind = "practice" if result["event"] == "practice" else "events"
    group = result["track"] if kind == "practice" else result["event"]
    return ROOT / kind / group / f"{result['team'].replace(' ', '_')}-{result['id']}.json"


def save(result):
    path = path_for(result)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=1) + "\n")
    return path


def load_all(root=ROOT):
    return [json.loads(p.read_text()) for p in sorted(pathlib.Path(root).rglob("*.json"))]


def validate(result):
    """Raise ValueError if a result file is malformed (used by the PR check)."""
    required = {"schema": int, "id": str, "event": str, "team": str, "track": str, "status": str,
                "lap_times": list, "collisions": int, "started_at": str}
    for key, typ in required.items():
        if not isinstance(result.get(key), typ):
            raise ValueError(f"field '{key}' missing or not {typ.__name__}")
    if result["schema"] != SCHEMA:
        raise ValueError(f"schema {result['schema']} not supported")
    if result["status"] not in ("finished", "dnf", "dsq", "error"):
        raise ValueError("bad status")
    dt.datetime.fromisoformat(result["started_at"])
    if any(not isinstance(t, (int, float)) or t <= 0 for t in result["lap_times"]):
        raise ValueError("lap_times must be positive numbers")
