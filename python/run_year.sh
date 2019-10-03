#!/usr/bin/env bash
RUN_SCRIPT=/home/pmateusz/dev/quake/python/year-tools.py
PLOT_SCRIPT=/home/pmateusz/dev/quake/python/plot-quake.py

#pushd /home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46/solutions
#$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46 --solution-prefix=solution --gap-limit=0.005 --time-step=00:00:15
#$PLOT_SCRIPT service-level --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46/solutions --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46
#$PLOT_SCRIPT network-traffic --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46/solutions
#$PLOT_SCRIPT long-term-performance --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46/solutions
#$PLOT_SCRIPT week-performance /home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46/year_2013.json /home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46/solutions/solution_2013-01-01.json
#$PLOT_SCRIPT aggregate --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46
#$PLOT_SCRIPT communication-window --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma500_raan110.5_ta46
#$PLOT_SCRIPT print-service-levels
#popd

#pushd /home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42/solutions
#$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42 --solution-prefix=solution --gap-limit=0.005 --time-step=00:00:15
#$PLOT_SCRIPT service-level --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42/solutions
#$PLOT_SCRIPT network-traffic --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42/solutions
#$PLOT_SCRIPT long-term-performance --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42/solutions
#$PLOT_SCRIPT week-performance /home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42 /home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42/solutions
#$PLOT_SCRIPT aggregate --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42
#$PLOT_SCRIPT communication-window --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta42
#$PLOT_SCRIPT print-service-levels
#popd

#pushd /home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280/solutions
##$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280 --solution-prefix=solution --gap-limit=0.005 --time-step=00:00:15
##$PLOT_SCRIPT service-level --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280/solutions
##$PLOT_SCRIPT network-traffic --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280/solutions
##$PLOT_SCRIPT long-term-performance --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280/solutions
#$PLOT_SCRIPT week-performance /home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280/year_2013.json home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280/solutions
#$PLOT_SCRIPT aggregate --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280
#$PLOT_SCRIPT communication-window --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan107_ta280
#$PLOT_SCRIPT print-service-levels
#popd

pushd /home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5/solutions
#$RUN_SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5 --solution-prefix=solution --gap-limit=0.005 --time-step=00:00:15
$PLOT_SCRIPT service-level --data-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5 --solution-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5/solutions
#$PLOT_SCRIPT network-traffic --data-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5 --solution-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5/solutions
#$PLOT_SCRIPT long-term-performance --data-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5 --solution-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5/solutions
#$PLOT_SCRIPT week-performance /home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5/year_2013.json /home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5/solutions
#$PLOT_SCRIPT aggregate --data-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5
#$PLOT_SCRIPT communication-window --data-dir=/home/pmateusz/dev/quake/current_review/sma566_inc97.4_ta0_raan110.5
$PLOT_SCRIPT print-service-levels
popd

#pushd /home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280/solutions
##$SCRIPT run --problem-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan110.5_ta46 --solution-prefix=solution --gap-limit=0.005 --time-step=00:00:15
#$PLOT_SCRIPT service-level --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280/solutions
##$PLOT_SCRIPT network-traffic --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280/solutions
##$PLOT_SCRIPT long-term-performance --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280 --solution-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280/solutions
##$PLOT_SCRIPT week-performance --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280/year_2013.json /home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280/solutions/solution_2013-01-01.json
##$PLOT_SCRIPT aggregate --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280
##$PLOT_SCRIPT communication-window --data-dir=/home/pmateusz/dev/quake/current_review/validation_sma566_raan118_ta280
#$PLOT_SCRIPT print-service-levels
#popd

# generate files with elevation angle
# /home/pmateusz/dev/quake/python/year-simulation.py elevation --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade

#/home/pmateusz/dev/quake/python/year-simulation.py run --data-dir=/home/pmateusz/dev/quake/data/simulation_upgrade --problem-dir=/home/pmateusz/dev/quake/data/simulation_upgrade_disturbed_10 --solution-prefix=solution_upgrade_disturbed_10
