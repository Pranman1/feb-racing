#!/bin/bash
# Installs scipy and CasADi into stack/.pydeps, next to your packages, where the racer's
# launch file finds them. Run once from the repo folder while the devkit is running
# (./feb-sim run in another terminal), or with the devkit on its own. Takes a minute.
set -e
docker exec feb-devkit bash -c 'pip install --quiet --no-deps --target /home/autodrive_devkit/src/stack/.pydeps "scipy==1.11.4" casadi && echo "deps installed in stack/.pydeps"'
