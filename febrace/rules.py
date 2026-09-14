"""Scoring rules, the same as the RoboRacer Sim Racing League used them in 2026.

A time-attack attempt is one warm-up lap followed by `laps` timed laps. Total time is the
sum of the timed laps plus `collision_penalty_s` per collision counted after the timer
starts. Attempts that do not complete all laps before `timeout_s` are DNF and rank
after every finisher, by laps completed. More than `max_collisions` collisions is DSQ.
"""
from dataclasses import asdict, dataclass

# To enter the first competition a team must, on EVERY practice track, complete a full
# attempt (all timed laps) with at most this many collisions. Verified (rerun) results count.
QUALIFY_MAX_COLLISIONS = 1


@dataclass
class Rules:
    laps: int = 10
    warmup_laps: int = 1
    collision_penalty_s: float = 10.0
    max_collisions: int = 10
    timeout_s: float = 300.0

    @property
    def total_laps(self):
        return self.warmup_laps + self.laps

    def to_dict(self):
        return asdict(self)


def score(rules, lap_times, collisions):
    """Return (status, timed_laps, penalty_s, total_s, best_lap_s)."""
    timed = [t for t in lap_times[rules.warmup_laps:] if t is not None][: rules.laps]
    penalty = rules.collision_penalty_s * collisions
    best = min(timed) if timed else None
    if collisions > rules.max_collisions:
        return "dsq", timed, penalty, None, best
    if len(timed) < rules.laps:
        return "dnf", timed, penalty, None, best
    return "finished", timed, penalty, round(sum(timed) + penalty, 3), best


def rank_key(result):
    """Sort key: finishers by total time, then DNFs by laps completed, then DSQs."""
    order = {"finished": 0, "dnf": 1, "dsq": 2}.get(result.get("status"), 3)
    return (order, result.get("total_s") or float("inf"), -len(result.get("lap_times") or []),
            result.get("best_lap_s") or float("inf"))


def qualifies(result):
    """A practice attempt that meets the qualification bar: finished, at most QUALIFY_MAX_COLLISIONS."""
    return result.get("status") == "finished" and result.get("collisions", 99) <= QUALIFY_MAX_COLLISIONS


def qualification(team, track_ids, practice_results):
    """Per-track status for a team: 'verified', 'unverified' (self-reported only) or None; and
    whether the team is qualified (verified on every track)."""
    status = {}
    for track in track_ids:
        rows = [r for r in practice_results if r["team"] == team and r["track"] == track and qualifies(r)]
        status[track] = "verified" if any(r.get("verified") for r in rows) else ("unverified" if rows else None)
    return status, all(v == "verified" for v in status.values()) and bool(track_ids)
