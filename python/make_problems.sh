#!/usr/bin/env bash

WEATHER_TOOLS_EXEC=/home/pmateusz/dev/quake/python/weather-tools.py

for scenario_generator in coregionalization past_error_replication correlated_error_simulation # independent_error_simulation correlated_error_simulation
do
$WEATHER_TOOLS_EXEC generate --from=2019-06-18 --to=2019-08-4 --num-scenarios=2048 --problem-prefix=problem_$scenario_generator --scenario-generator=$scenario_generator
done