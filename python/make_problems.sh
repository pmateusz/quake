#!/usr/bin/env bash

WEATHER_TOOLS_EXEC=/home/pmateusz/dev/quake/python/weather-tools.py

for scenario_generator in past_error_replication
do
$WEATHER_TOOLS_EXEC generate --from=2019-07-01 --to=2019-09-1 --num-scenarios=256 --problem-prefix=problem --scenario-generator=$scenario_generator
done