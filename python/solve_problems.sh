#!/usr/bin/env bash

pushd /home/pmateusz/dev/quake/network_share/test/solution_dir

/home/pmateusz/dev/quake/cmake-build-debug/quake-det --problem=../problem_dir/problem_past_error_replication_2019-06-18.json --gap-limit=0.05 \
--solution-prefix=solution_error_replication_2019-06-18 --interval-step=00:01:00 > solution_error_replication_2019-06-18.log 2> solution_error_replication_2019-06-18.err.log

popd

