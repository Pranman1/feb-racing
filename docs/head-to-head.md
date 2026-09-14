# Head-to-head races

Two or more cars race in one simulator at the same time. Every team's container still sees
its own car as `roboracer_1`: a small proxy (`febrace/proxy.py`, run inside the devkit image)
sits between the simulator and the containers and rewrites the vehicle ids, so a container
that works in time-attack works unchanged in head-to-head.

```
simulator --cars 2 --connect :4570  <->  proxy (port 4570)  <->  container A (port 4568, sees V1)
                                                            <->  container B (port 4569, sees V1)
```

The official AutoDRIVE Race Control Tower does the same rewriting and adds stewarding: after
any car-to-car contact it freezes both cars until a race director rules in its web UI. That
is right for a live final with a human steward (build it with
`docker build -t feb-rct https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Race-Control-Tower.git`
and point the simulator at it) and wrong for automated brackets, which is why the harness
uses its own proxy.

## One race

```bash
./feb-race h2h --teams "Team A" "Team B" --track tracks/loop --laps 10 --gui
```

Cars start on a staggered grid centred on the spawn pose: slots 0.6 m apart sideways and
each further slot 1 m behind the previous one. Each car's laps and collisions are probed in its own container;
the result files carry `mode: head-to-head`, `race`, `opponents`, `finish_s` (time of the last
lap plus 10 s per collision) and `position` (most laps first, then finish time). Car-to-car
contact counts as a collision for both cars and respawns them side by side, as upstream does.

## Tournament

```bash
./feb-race bracket 2026-11-final --seed-from 2026-10-qualification
```

Seeds come from the time-attack event's standings. Single elimination, 1 v N, 2 v N-1, and
so on; with an odd count the top seed has a bye. Race ids are `r<round>-m<match>`; the site
shows every race under the event and ranks teams by the round they reached.

## More than two cars

`--cars N` is supported end to end (the simulator clones the car, the proxy maps N containers)
and a three-car race completes its laps, but drivers do not see each other, so contact and
pair respawns pile up. Treat 3+ cars as experimental until the rules for multi-car contact
are written; two cars is the supported race mode.

## Practice

Members race the house racer locally: `./feb-sim run --stack "..." --opponent normal`. The house
racer is the smooth lidar driver from the starter kit at a conservative pace (`slow` 1.5 m/s,
`normal` 2.0, `fast` 2.5; zero wall contact over eight laps on the loop at 10 Hz), the thing a new
stack has to overtake. `--opponent map` runs the map-following driver instead (it knows the track
and the car's true pose; faster, less smooth). `--cars 2` alone puts an undriven second car on the
grid. Tracks with `max_cars: 1` (Porto: 1.3 m corridor) refuse a second car.
