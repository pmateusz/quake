#!/usr/bin/env bash

#/home/pmateusz/dev/quake/python/year-simulation.py generate --year=2018 --output=/home/pmateusz/dev/quake/data/simulation_upgrade

# generate files with elevation angle
# /home/pmateusz/dev/quake/python/year-simulation.py elevation --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade

/home/pmateusz/dev/quake/python/year-simulation.py run --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade --problem-dir=/home/pmateusz/dev/quake/data/simulation_upgrade_disturbed_10 --solution-prefix=solution_upgrade_disturbed_10
