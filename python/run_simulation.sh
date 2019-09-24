#!/usr/bin/env bash

/home/pmateusz/dev/quake/python/weather-tools.py generate --problem-prefix=week --from=2013-01-01 --to=2014-01-01 --time-horizon="7 days" --initial-epoch=2012-12-22

#/home/pmateusz/dev/quake/python/year-simulation.py generate --year=2014 --output=/home/pmateusz/dev/quake/current_review
#/home/pmateusz/dev/quake/python/year-simulation.py generate --year=2015 --output=/home/pmateusz/dev/quake/current_review
#/home/pmateusz/dev/quake/python/year-simulation.py generate --year=2016 --output=/home/pmateusz/dev/quake/current_review
#/home/pmateusz/dev/quake/python/year-simulation.py generate --year=2017 --output=/home/pmateusz/dev/quake/current_review
#/home/pmateusz/dev/quake/python/year-simulation.py generate --year=2018 --output=/home/pmateusz/dev/quake/current_review

# generate files with elevation angle
# /home/pmateusz/dev/quake/python/year-simulation.py elevation --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade

#/home/pmateusz/dev/quake/python/year-simulation.py run --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade --problem-dir=/home/pmateusz/dev/quake/data/simulation_upgrade_disturbed_10 --solution-prefix=solution_upgrade_disturbed_10
