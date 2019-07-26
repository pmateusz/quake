#!/usr/bin/env bash

WEATHER_TOOLS_EXEC=/home/pmateusz/dev/quake/python/weather-tools.py

for scenario_generator in past_error_replication # independent_error_simulation correlated_error_simulation coregionalization
do
$WEATHER_TOOLS_EXEC generate --from=2019-06-18 --to=2019-07-25 --num-scenarios=2048 --problem-prefix=problem_$scenario_generator --scenario-generator=$scenario_generator
done