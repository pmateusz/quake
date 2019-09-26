#!/usr/bin/env bash

WEATHER_TOOLS_EXEC=/home/pmateusz/dev/quake/python/weather-tools.py

#pushd /home/pmateusz/dev/quake/current_review/2013
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2013-01-01 --to=2014-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2012-12-22
#popd

pushd /home/pmateusz/dev/quake/current_review/2014
$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2014-01-01 --to=2015-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2013-12-22
popd


#for scenario_generator in past_error_replication
#do
#$WEATHER_TOOLS_EXEC generate --from=2019-07-01 --to=2019-09-1 --num-scenarios=256 --problem-prefix=problem --scenario-generator=$scenario_generator
#done