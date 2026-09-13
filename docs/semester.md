# A semester with the simulator

Members learn by tuning against a leaderboard; the organiser reviews rather than lectures.

| When | What | Where it shows |
|---|---|---|
| Week 0 | Install, drive manually, run the starter driver, post one practice result | everyone on the `loop` practice board |
| Weeks 1 to 5 | Self-paced stages 1 to 4 (`docs/starter-kit.md`), one practice track per stage | practice boards, nightly verified reruns |
| Mid-semester | Qualification: time-attack on a secret track, 2 attempts, best counts | `events/<qualification>` |
| Weeks 7 to 12 | Weekly open rounds on rotating practice tracks; released competition tracks join the pool | practice boards |
| Finals | Time-attack on a new secret track for seeding, then a head-to-head knockout | `events/<final>` with races |

Cadence: one attempt takes about 3 minutes on the organiser PC, so a nightly rerun of 10
teams on 3 tracks is under two hours and a live event of 10 teams is under an hour.

Releasing tracks: a competition track is secret only until its event. Afterwards it moves
into `tracks/`, ships with the next simulator release and is practised on. New events get
new tracks; designing one takes minutes (`docs/tracks.md`).

Roles: the organiser owns the secret tracks, the events folder and the PC; members own
their `stack/`, their images and their practice results.
