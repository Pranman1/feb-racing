# Head-to-head races

Two or more cars race in one simulator at the same time. Every team's container still sees
its own car as `roboracer_1`: the official AutoDRIVE Race Control Tower (RCT) proxy sits
between the simulator and the containers and rewrites the vehicle ids, so a container that
works in time-attack works unchanged in head-to-head.

```
simulator --cars 2 --connect :4570  <->  RCT (port 4570)  <->  container A (port 4568, sees V1)
                                                          <->  container B (port 4569, sees V1)
```

## Setup (organiser PC, once)

```bash
docker build -t feb-rct https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Race-Control-Tower.git
```

## One race

```bash
./feb-race h2h --teams "Team A" "Team B" --track tracks/loop --laps 10 --gui
```

Both cars spawn side by side on the start line (car 0 on the centreline, the others 0.35 m
left and right, alternating). Each car's laps and collisions are probed in its own container;
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

`--cars N` is supported end to end (simulator clones the car, the RCT maps N containers).
Expect the simulator frame rate to drop past 4 cars, and write the rules for multi-car
contact before you use it for anything that counts.

## Practice

Members can put a second car on the grid locally: `./feb-sim run --cars 2`. The second car
is driven by nobody unless a second bridge feeds it; it still makes a useful static obstacle.
