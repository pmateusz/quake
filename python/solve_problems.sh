#!/usr/bin/env bash

problem_dir=/home/pmateusz/dev/quake/network_share/test/problem_dir
solution_dir=/home/pmateusz/dev/quake/network_share/test/solution_dir
interval_step=00:01:00
gap_limit=0.05
num_scenarios=128
time_limit=00:05:00


pushd $solution_dir || exit

function run_deterministic {
  # args: base_file_name date
  output=solution\_$1\_$2
  output_log=$output\_deterministic
  /home/pmateusz/dev/quake/cmake-build-debug/quake-det --problem=../problem_dir/problem\_$1\_$2\.json --interval-step=${interval_step} \
--gap-limit=${gap_limit} --time-limit=${time_limit} --solution-prefix=${output} > ${output_log}\.log 2> ${output_log}\.err.log
}
#
#pattern=".*/problem_(.*)_([0-9]{4}\-[0-9]{2}\-[0-9]{2}).json$"
#for problem_file in /home/pmateusz/dev/quake/network_share/test/problem_dir/problem_coregionalization*.json; do
#  if [[ $problem_file =~ ${pattern} ]]
#  then
#    base_filename_match=${BASH_REMATCH[1]}
#    date_match=${BASH_REMATCH[2]}
#    echo Solving deterministic $base_filename_match $date_match
#    run_deterministic $base_filename_match $date_match
#  fi
#done

function run_saa {
  base_file_name=$1
  date=$2
  target_traffic_index=$3
  output_file=solution\_${base_file_name}\_${date}\_$num_scenarios\_${target_traffic_index}\_saa
  /home/pmateusz/dev/quake/cmake-build-debug/quake-saa --problem=../problem_dir/problem\_${base_file_name}\_${date}\.json --interval-step=$interval_step --gap-limit=$gap_limit --time-limit=${time_limit} \
--target-traffic-index=${target_traffic_index} --num_scenarios=$num_scenarios --output=${output_file}\.json > ${output_file}.log 2> ${output_file}.err.log
}

function run_cvar {
  base_file_name=$1
  date=$2
  target_traffic_index=$3
  epsilon=$4
  output_file=solution\_${base_file_name}\_${date}\_$num_scenarios\_${target_traffic_index}\_${epsilon//./}\_cvar
  /home/pmateusz/dev/quake/cmake-build-debug/quake-cvar --problem=../problem_dir/problem\_$1\_$2\.json --interval-step=${interval_step} --gap-limit=${gap_limit} --time-limit=${time_limit} \
--target-traffic-index=$3 --num_scenarios=$num_scenarios --epsilon=${epsilon} --output=${output_file}\.json > ${output_file}.log 2> ${output_file}.err.log
}

# solve single day using saa and different values of the target traffic index
#for target_traffic_index in 3000 4000 5000 6000 7000
#do
#base_filename=coregionalization
#date=2019-07-18
#echo Solving sample average approximation ${base_filename} ${date} ${num_scenarios} ${target_traffic_index}
#run_saa ${base_filename} ${date} ${target_traffic_index}
#done

# solve single day using cvar and different values of the target traffic index
for target_traffic_index in 3000 4000 5000 6000 7000
do
base_filename=coregionalization
date=2019-07-18
epsilon=0.05
echo Solving CV@R ${base_filename} ${date} ${num_scenarios} ${target_traffic_index} ${epsilon}
run_cvar ${base_filename} ${date} ${target_traffic_index} ${epsilon}
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
