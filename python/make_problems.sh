#!/usr/bin/env bash

WEATHER_TOOLS_EXEC=/home/pmateusz/dev/quake/python/weather-tools.py
PROBLEM_GENERATOR_EXEC=/home/pmateusz/dev/quake/build/quake-generate
PLOT_SCRIPT=/home/pmateusz/dev/quake/python/plot-quake.py

set -e

pushd /home/pmateusz/dev/quake/current_review

for raan in $(seq 90.5 1 116)
do

  mkdir -p $raan
  pushd $raan

  for year in $(seq 2013 1 2018)
  do
    $PROBLEM_GENERATOR_EXEC --altitude=566.897046176199410 --inclination=97.658190099944605 --raan=$raan --from=$year-01-01 --to=$((year+1))-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_$year.json_temp
    $WEATHER_TOOLS_EXEC extend year_$year.json_temp --output=year_$year.json
    rm year_$year.json_temp
  done



#  $PROBLEM_GENERATOR_EXEC --from=2014-01-01 --to=2015-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2014.json_temp
#  $WEATHER_TOOLS_EXEC extend year_2014.json_temp --output=year_2014.json
#  rm year_2014.json_temp
#
#  $PROBLEM_GENERATOR_EXEC --from=2015-01-01 --to=2016-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2015.json_temp
#  $WEATHER_TOOLS_EXEC extend year_2015.json_temp --output=year_2015.json
#  rm year_2015.json_temp
#
#  $PROBLEM_GENERATOR_EXEC --from=2016-01-01 --to=2017-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2016.json_temp
#  $WEATHER_TOOLS_EXEC extend year_2016.json_temp --output=year_2016.json
#  rm year_2016.json_temp
#
#  $PROBLEM_GENERATOR_EXEC --from=2017-01-01 --to=2018-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2017.json_temp
#  $WEATHER_TOOLS_EXEC extend year_2017.json_temp --output=year_2017.json
#  rm year_2017.json_temp
#
#  $PROBLEM_GENERATOR_EXEC --from=2018-01-01 --to=2019-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2018.json_temp
#  $WEATHER_TOOLS_EXEC extend year_2018.json_temp --output=year_2018.json
#  rm year_2018.json_temp

  popd

done

popd

#pushd /home/pmateusz/dev/quake/current_review/2013
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2013-01-01 --to=2014-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2012-12-22
#popd

#pushd /home/pmateusz/dev/quake/current_review/2014
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2014-01-01 --to=2015-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2013-12-22
#popd

#pushd /home/pmateusz/dev/quake/current_review/2015
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2015-01-01 --to=2016-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2014-12-22
#popd

#pushd /home/pmateusz/dev/quake/current_review/2016
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2016-01-01 --to=2017-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2015-12-22
#popd

#pushd /home/pmateusz/dev/quake/current_review/2017
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2017-01-01 --to=2018-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2016-12-22
#popd
#
#pushd /home/pmateusz/dev/quake/current_review/2018
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2018-01-01 --to=2019-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2017-12-22
#popd

#pushd /home/pmateusz/dev/quake/current_review/test_sma500_raan112_ta51
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2013-01-01 --to=2014-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch=2013-01-01
#popd

#pushd /home/pmateusz/dev/quake/current_review/alt_566
#$WEATHER_TOOLS_EXEC generate --problem-prefix=week --from=2013-01-01 --to=2014-01-01 --time-horizon="7 days" --time-step="7 days" --initial-epoch="2013-01-01 00:00:00"
#popd

#pushd /home/pmateusz/dev/quake/current_review/beta_sso_j2
#$PROBLEM_GENERATOR_EXEC --from=2013-01-01 --to=2014-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2013.json_temp
#$WEATHER_TOOLS_EXEC extend year_2013.json_temp --output=year_2013.json
#
#$PROBLEM_GENERATOR_EXEC --from=2014-01-01 --to=2015-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2014.json_temp
#$WEATHER_TOOLS_EXEC extend year_2014.json_temp --output=year_2014.json
#
#$PROBLEM_GENERATOR_EXEC --from=2015-01-01 --to=2016-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2015.json_temp
#$WEATHER_TOOLS_EXEC extend year_2015.json_temp --output=year_2015.json
#
#$PROBLEM_GENERATOR_EXEC --from=2016-01-01 --to=2017-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2016.json_temp
#$WEATHER_TOOLS_EXEC extend year_2016.json_temp --output=year_2016.json
#
#$PROBLEM_GENERATOR_EXEC --from=2017-01-01 --to=2018-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2017.json_temp
#$WEATHER_TOOLS_EXEC extend year_2017.json_temp --output=year_2017.json
#
#$PROBLEM_GENERATOR_EXEC --from=2018-01-01 --to=2019-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2018.json_temp
#$WEATHER_TOOLS_EXEC extend year_2018.json_temp --output=year_2018.json

#$PROBLEM_GENERATOR_EXEC --from=2019-01-01 --to=2020-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2019.json
#$PROBLEM_GENERATOR_EXEC --from=2020-01-01 --to=2021-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2020.json
#$PROBLEM_GENERATOR_EXEC --from=2021-01-01 --to=2022-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2021.json
#$PROBLEM_GENERATOR_EXEC --from=2022-01-01 --to=2023-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2022.json
#$PROBLEM_GENERATOR_EXEC --from=2023-01-01 --to=2024-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2023.json
#$PROBLEM_GENERATOR_EXEC --from=2024-01-01 --to=2025-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2024.json
#$PROBLEM_GENERATOR_EXEC --from=2025-01-01 --to=2026-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2025.json
#$PROBLEM_GENERATOR_EXEC --from=2026-01-01 --to=2027-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2026.json
#$PROBLEM_GENERATOR_EXEC --from=2027-01-01 --to=2028-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2027.json
#$PROBLEM_GENERATOR_EXEC --from=2028-01-01 --to=2029-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2028.json
#$PROBLEM_GENERATOR_EXEC --from=2029-01-01 --to=2030-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2029.json
#$PROBLEM_GENERATOR_EXEC --from=2030-01-01 --to=2031-01-01 --initial-epoch="2013-01-01 00:00:00" --output=year_2030.json
#popd

#for scenario_generator in past_error_replication
#do
#$WEATHER_TOOLS_EXEC generate --from=2019-07-01 --to=2019-09-1 --num-scenarios=256 --problem-prefix=problem --scenario-generator=$scenario_generator
#done