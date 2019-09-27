#!/usr/bin/env bash
SCRIPT=/home/pmateusz/dev/quake/python/year-tools.py

#pushd /home/pmateusz/dev/quake/current_review/2013/solutions
#$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/2013 --solution-prefix=solution --gap-limit=0.05 --time-step=00:00:15
#popd

#pushd /home/pmateusz/dev/quake/current_review/2014/solutions
#$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/2014 --solution-prefix=solution --gap-limit=0.05 --time-step=00:00:15
#popd

#pushd /home/pmateusz/dev/quake/current_review/2015/solutions
#$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/2015 --solution-prefix=solution --gap-limit=0.05 --time-step=00:00:15
#popd

#pushd /home/pmateusz/dev/quake/current_review/2016/solutions
#$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/2016 --solution-prefix=solution --gap-limit=0.05 --time-step=00:00:15
#popd

pushd /home/pmateusz/dev/quake/current_review/2017/solutions
$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/2017 --solution-prefix=solution --gap-limit=0.05 --time-step=00:00:15
popd

pushd /home/pmateusz/dev/quake/current_review/2018/solutions
$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/2018 --solution-prefix=solution --gap-limit=0.05 --time-step=00:00:15
popd

# generate files with elevation angle
# /home/pmateusz/dev/quake/python/year-simulation.py elevation --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade

#/home/pmateusz/dev/quake/python/year-simulation.py run --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade --problem-dir=/home/pmateusz/dev/quake/data/simulation_upgrade_disturbed_10 --solution-prefix=solution_upgrade_disturbed_10
