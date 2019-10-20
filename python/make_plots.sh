#!/usr/bin/env bash

PLOT_TOOL=/home/pmateusz/dev/quake/python/plot-quake.py
PROBLEM_DIR=/home/pmateusz/dev/quake/current_review/109.5

set -e

mkdir -p /home/pmateusz/dev/quake/current_review/plots

pushd /home/pmateusz/dev/quake/current_review/plots
$PLOT_TOOL aggregate --data-dir=$PROBLEM_DIR
$PLOT_TOOL communication-window --data-dir=$PROBLEM_DIR
popd


#/home/pmateusz/dev/quake/python/plot-quake.py week-performance /home/pmateusz/dev/quake/current_review/2013/week_2013-01-01.json /home/pmateusz/dev/quake/current_review/2013/solutions/solution_2013-01-01.json # ported
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/current_review/2013 --solution-dir=/home/pmateusz/dev/quake/current_review/2013/solutions # ported
#/home/pmateusz/dev/quake/python/plot-quake.py network-traffic --data-dir=/home/pmateusz/dev/quake/current_review/2013 --solution-dir=/home/pmateusz/dev/quake/current_review/2013/solutions # ported
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/network_share/simulation_upgrade --solution-dir=/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_1
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_11_mip
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_12
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_13
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_14
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_15
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_16
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_17
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_18
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_20
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_1 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_21_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_2 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_22_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_3 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_23_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_4 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_24_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py long-term-performance --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_5 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_25_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_12
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_13
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_14
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_15
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_16
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_17
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_18
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_19
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_20
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_1 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_21_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_2 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_22_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_3 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_23_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_4 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_24_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_5 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_25_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_6 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_26_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_7 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_27_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_8 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_28_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_9 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_29_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py service-level --data-dir=/home/pmateusz/dev/quake/data/simulation_disturbed_10 --solution-dir=/home/pmateusz/dev/quake/data/simulation/run_30_disturbed
#/home/pmateusz/dev/quake/python/plot-quake.py switch-time-performance "/home/pmateusz/dev/quake/network_share/simulation/evaluation/solution_2s_*.json" --output=optics_performance_2s
#/home/pmateusz/dev/quake/python/plot-quake.py switch-time-performance "/home/pmateusz/dev/quake/network_share/simulation/evaluation/solution_4s_*.json" --output=optics_performance_4s
#/home/pmateusz/dev/quake/python/plot-quake.py jobs-performance "/home/pmateusz/dev/quake/network_share/simulation/evaluation/solution_2s_30_jobs*.json" --output=bundle_2s_1sec_30switch --max-jobs=8
#/home/pmateusz/dev/quake/python/plot-quake.py jobs-performance "/home/pmateusz/dev/quake/network_share/simulation/evaluation/solution_4s_30_jobs*.json" --output=bundle_4s_30switch --max-jobs=9
#/home/pmateusz/dev/quake/python/plot-quake.py weights-disturbed --data-dir=/home/pmateusz/dev/quake/network_share
#/home/pmateusz/dev/quake/python/plot-quake.py all-service-level
#/home/pmateusz/dev/quake/python/plot-quake.py key-rate

#extension=pdf
#
#mkdir -p export
#mv average_cloud_cover.$extension export/
##mv bundle_2s_1sec_30switch.$extension export/
##mv bundle_4s_30switch.$extension export/
#mv cloud_cover_correlation.$extension export/
#mv cumulative_observation_time_London.$extension export/
##mv long_term_lambda_run_1.$extension export/
#mv long_term_performance_v2_run_1.$extension export/long_term_performance.$extension
##mv long_term_performance_run_1.$extension export/
#mv maximum_keys_correlation.$extension export/
#mv maximum_keys_received.$extension export/
#mv network_traffic.$extension export/
#mv noisy_weight_boxplot_10_London.$extension export/
#mv observation_time.$extension export/
#mv observation_time_London.$extension export/
#mv observation_time_magnified_London.$extension export/
##mv optics_performance_4s.$extension export/
##mv service_level_global_run1.$extension export/
##mv service_level_global_run_perturbed10.$extension export/
#mv service_level_London_run1.$extension export/service_level_London.$extension
##mv solution_week_2018-01-01.$extension export/
#mv solution_week_v2_2018-01-01.$extension export/solution_week_2018-01-01.$extension
#mv key_rate.$extension export/
#popd