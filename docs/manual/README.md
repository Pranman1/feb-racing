# Manuals

- `competitor.pdf`: for a new member, from install to a leaderboard entry and a competition image.
- `competitor_mac.pdf`: the same for a Mac, every step from an empty machine (Docker Desktop, Rosetta, Foxglove); standalone, built with `tectonic competitor_mac.tex`.
- `organiser.pdf`: for the organiser, from tracks and events to race day, releases and maintenance.

Both share `preamble.tex` and `platform.tex` (the platform and the rules). Rebuild:

```bash
docker run --rm -v "$PWD/../..:/work" -w /work/docs/manual texlive/texlive:latest-medium \
  bash -c 'for m in competitor organiser; do pdflatex $m.tex && pdflatex $m.tex; done; rm -f *.aux *.log *.out *.toc'
```
