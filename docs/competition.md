# Running a competition (organiser)

Everything official runs on the team PC with the headless Linux simulator. Members never
need access to it.

## Rules (from the RoboRacer Sim Racing League)

- One attempt = 1 warm-up lap + 10 timed laps. Total = sum of the timed laps + 10 s per collision.
- Not finishing within 300 s is DNF (ranked after every finisher, by laps completed). More than
  10 collisions is DSQ. Best of `attempts` per team counts.
- Ground-truth topics (`ips`, `odom`, `tf`, lap and collision counters, `reset_command`) are for
  training and debugging only. The harness audits which nodes subscribe to them during the run and
  flags the result (`restricted_subscribers`); the site shows a red badge.
- Everything a submission needs must start from the container's entrypoint
  (`/home/autodrive_devkit.sh`), exactly as in the league.

## An event, step by step

1. Design a secret track in a private folder (`docs/tracks.md`).
2. Add `events/<id>.yaml` (copy `events/2026-10-qualification.yaml`), commit, push: the site lists it.
3. Teams register their image in `submissions.yaml` (pull request) and push it to a public registry.
4. On the day:
   ```bash
   ./feb-race event <id>          # pulls every image, runs each attempt, prints the standings
   git add results && git commit -m "<id> results" && git push     # the site updates itself
   ```
   Each attempt leaves `runs/<id>/` with `result.json`, `sim.log`, `container.log` and a bag.
5. Set `status: done` in the event file and move the track into `tracks/`.

## Practice leaderboard

- Members post their own attempts with `feb-sim submit` (a pull request that merges itself
  once `.github/workflows/results-pr.yml` validates it). They show as *unverified*.
- `./feb-race verify results/practice/<track>/<file>.json` reruns that image here and posts a
  verified result; `./feb-race nightly` reruns every registered image on every practice track.
  Put the nightly in cron on the PC, then commit and push `results/`.

## Head-to-head

See `docs/head-to-head.md`. Short version: `./feb-race bracket <final-event> --seed-from <qualification-event>`.

## Where things run

| Piece | Where |
|---|---|
| Simulator | native app: `~/.feb-sim/app/linux/FEB Simulator.x86_64` (`FEB_SIM_APP` overrides) |
| Team container | `docker run -p <port>:4567 <image>`; the harness probes and records bags inside it |
| Results | `results/events/<event>/` and `results/practice/<track>/`, one JSON per attempt |
| Site | GitHub Pages, rebuilt by `.github/workflows/site.yml` on every push to `main` |
