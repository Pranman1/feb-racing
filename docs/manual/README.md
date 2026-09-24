# Manuals

- `competitor.pdf`: for a new member, from install to a leaderboard entry and a competition image.
- `competitor_setup_mac.pdf`, `competitor_setup_windows.pdf`, `competitor_setup_linux.pdf`: setup only, every step from an empty machine to the first autonomous lap, one per system. Shared body `setup_body.tex`, per-system install `setup_install_<os>.tex`, shared appendix `starter_driver.tex`. Build: `tectonic competitor_setup_<os>.tex`.
- `organiser.pdf`: for the organiser, from tracks and events to race day, releases and maintenance.

Both share `preamble.tex` and `platform.tex` (the platform and the rules). Rebuild:

```bash
docker run --rm -v "$PWD/../..:/work" -w /work/docs/manual texlive/texlive:latest-medium \
  bash -c 'for m in competitor organiser; do pdflatex $m.tex && pdflatex $m.tex; done; rm -f *.aux *.log *.out *.toc'
```
