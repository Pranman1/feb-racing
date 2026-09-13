"""Build the static leaderboard site: site/_build/{index.html, data.json, tracks/*.png}.

data.json holds everything the page renders: teams, tracks, events, and per-track /
per-event standings. Practice standings prefer verified results; self-reported ones are
shown with an "unverified" badge.
"""
import json
import pathlib
import shutil

import yaml

from . import results, rules as rules_mod

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "site" / "_build"


def standings(rows):
    """Best result per team: verified beats unverified, then the rank key."""
    best = {}
    for r in sorted(rows, key=lambda r: (not r.get("verified"), rules_mod.rank_key(r))):
        best.setdefault(r["team"], r)
    return sorted(best.values(), key=lambda r: (not r.get("verified"), rules_mod.rank_key(r)))


def build():
    teams = yaml.safe_load((ROOT / "submissions.yaml").read_text())["teams"]
    events = [yaml.safe_load(p.read_text()) for p in sorted((ROOT / "events").glob("*.yaml"))]
    all_results = results.load_all()

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "tracks").mkdir(exist_ok=True)
    tracks = []
    for folder in sorted((ROOT / "tracks").glob("*/track.json")):
        t = json.loads(folder.read_text())
        name = folder.parent.name
        preview = folder.with_name("preview.png")
        if preview.exists():
            shutil.copy(preview, OUT / "tracks" / f"{name}.png")
        rows = [r for r in all_results if r["event"] == "practice" and r["track"] == name]
        tracks.append({"id": name, "name": t["name"], "length": t["length"], "direction": t["direction"],
                       "walls": bool(t["walls"]), "cones": bool(t["cones"]), "preview": preview.exists(),
                       "standings": standings(rows), "attempts": len(rows)})

    for e in events:
        rows = [r for r in all_results if r["event"] == e["id"]]
        e["standings"] = standings(rows)
        e["attempts"] = len(rows)
        e["track"] = pathlib.Path(str(e["track"])).name

    data = {"teams": teams, "tracks": tracks, "events": events,
            "generated": __import__("datetime").datetime.now(__import__("datetime").timezone.utc).isoformat(timespec="seconds")}
    (OUT / "data.json").write_text(json.dumps(data, indent=1, default=str))
    shutil.copy(ROOT / "site" / "index.html", OUT / "index.html")
    shutil.copy(ROOT / "assets" / "feb_logo.svg", OUT / "logo.svg")
    return OUT


if __name__ == "__main__":
    print(build())
