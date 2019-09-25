#!/usr/bin/env bash
SCRIPT=/home/pmateusz/dev/quake/python/year-tools.py

pushd /home/pmateusz/dev/quake/current_review/2013/solutions
$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review --solution-prefix=solution --gap-limit=0.05 --time-step=00:00:15
popd

# generate files with elevation angle
# /home/pmateusz/dev/quake/python/year-simulation.py elevation --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade

#/home/pmateusz/dev/quake/python/year-simulation.py run --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade --problem-dir=/home/pmateusz/dev/quake/data/simulation_upgrade_disturbed_10 --solution-prefix=solution_upgrade_disturbed_10
