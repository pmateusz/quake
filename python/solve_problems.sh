#!/usr/bin/env bash

problem_dir=/home/pmateusz/dev/quake/network_share/test/problem_dir
solution_dir=/home/pmateusz/dev/quake/network_share/test/solution_dir
interval_step=00:01:00
gap_limit=0.05
num_scenarios=128


pushd $solution_dir || exit

pattern=".*/(.*)_([0-9]{4}\-[0-9]{2}\-[0-9]{2}).json$"
deterministic_exec=/home/pmateusz/dev/quake/cmake-build-debug/quake-det
saa_exec=/home/pmateusz/dev/quake/cmake-build-debug/quake-saa

#for problem_file in /home/pmateusz/dev/quake/network_share/test/problem_dir/problem_coregionalization*.json; do
#  if [[ $problem_file =~ $pattern ]]
#  then
#    base_filename_match=${BASH_REMATCH[1]}
#    date_match=${BASH_REMATCH[2]}
#    echo Solving deterministic $base_filename_match $date_match
#    $deterministic_exec --problem=$problem_file --interval-step=$interval_step --gap-limit=$gap_limit \
#    --solution-prefix=$base_filename_match\_$date_match > $base_filename_match\_$date_match\.log 2> $base_filename_match\_$date_match\.err.log
#  fi
#done

for index in 3000 4000 5000 6000 7000
do
base_filename_match=problem_coregionalization
target_traffic_index=$index
date_match=2019-07-18
output=$base_filename_match\_$date_match\_$target_traffic_index\_saa
$saa_exec --problem=../problem_dir/$base_filename_match\_$date_match\.json --interval-step=$interval_step --gap-limit=$gap_limit \
 --target-traffic-index=$target_traffic_index --num_scenarios=$num_scenarios --output=$output > $output.log 2> $output.err.log
done

#/home/pmateusz/dev/quake/cmake-build-debug/quake-det

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=6000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_6000.json > solution_error_replication_saa_2019-06-18_6000.log 2> solution_error_replication_saa_2019-06-18_6000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=9000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_9000.json > solution_error_replication_saa_2019-06-18_9000.log 2> solution_error_replication_saa_2019-06-18_9000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12000.json > solution_error_replication_saa_2019-06-18_12000.log 2> solution_error_replication_saa_2019-06-18_12000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=16000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_16000.json > solution_error_replication_saa_2019-06-18_16000.log 2> solution_error_replication_saa_2019-06-18_16000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=14000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_14000.json > solution_error_replication_saa_2019-06-18_14000.log 2> solution_error_replication_saa_2019-06-18_14000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=13000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_13000.json > solution_error_replication_saa_2019-06-18_13000.log 2> solution_error_replication_saa_2019-06-18_13000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12500 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12500.json > solution_error_replication_saa_2019-06-18_12500.log 2> solution_error_replication_saa_2019-06-18_12500.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12300 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12300.json > solution_error_replication_saa_2019-06-18_12300.log 2> solution_error_replication_saa_2019-06-18_12300.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12400 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12400.json > solution_error_replication_saa_2019-06-18_12400.log 2> solution_error_replication_saa_2019-06-18_12400.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12350 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12350.json > solution_error_replication_saa_2019-06-18_12350.log 2> solution_error_replication_saa_2019-06-18_12350.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12325 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12325.json > solution_error_replication_saa_2019-06-18_12325.log 2> solution_error_replication_saa_2019-06-18_12325.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12335 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12335.json > solution_error_replication_saa_2019-06-18_12335.log 2> solution_error_replication_saa_2019-06-18_12335.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12340 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12340.json > solution_error_replication_saa_2019-06-18_12340.log 2> solution_error_replication_saa_2019-06-18_12340.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12338 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12338.json > solution_error_replication_saa_2019-06-18_12338.log 2> solution_error_replication_saa_2019-06-18_12338.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=12336 --num-scenarios=128 --output=solution_error_replication_2019-06-18_saa_128_12336.json > solution_error_replication_saa_2019-06-18_12336.log 2> solution_error_replication_saa_2019-06-18_12336.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-cvar --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=3000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_cvar_128_3000.json > solution_error_replication_cvar_2019-06-18_3000.log 2> solution_error_replication_cvar_2019-06-18_3000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-cvar --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=6000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_cvar_128_6000.json > solution_error_replication_cvar_2019-06-18_6000.log 2> solution_error_replication_cvar_2019-06-18_6000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-cvar --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=4000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_cvar_128_4000.json > solution_error_replication_cvar_2019-06-18_4000.log 2> solution_error_replication_cvar_2019-06-18_4000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-cvar --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#--target-traffic-index=5000 --num-scenarios=128 --output=solution_error_replication_2019-06-18_cvar_128_5000.json > solution_error_replication_cvar_2019-06-18_5000.log 2> solution_error_replication_cvar_2019-06-18_5000.err.log

#/home/pmateusz/dev/quake/cmake-build-debug/quake-cvar --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --interval-step=00:01:00 --gap-limit=0.05 \
#  --target-traffic-index=12335 --epsilon=1.0 --num-scenarios=128 --output=solution_error_replication_2019-06-18_cvar_128_12335_e1.json >solution_error_replication_cvar_2019-06-18_12335e1.log 2>solution_error_replication_cvar_2019-06-18_12335e1.err.log

popd
