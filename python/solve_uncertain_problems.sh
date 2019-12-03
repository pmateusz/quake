#!/usr/bin/env bash
SOLVER=/home/pmateusz/dev/quake/cmake-build-debug/quake-det
PROBLEM_DIR=/home/pmateusz/dev/quake/current_review

PROBLEMS=($PROBLEM_DIR/week_2019-10-01.json:$PROBLEM_DIR/week_2019-10-02.json:$PROBLEM_DIR/week_2019-10-03.json:$PROBLEM_DIR/week_2019-10-04.json:$PROBLEM_DIR/week_2019-10-05.json \
$PROBLEM_DIR/week_2019-10-06.json:$PROBLEM_DIR/week_2019-10-07.json:$PROBLEM_DIR/week_2019-10-08.json:$PROBLEM_DIR/week_2019-10-09.json:$PROBLEM_DIR/week_2019-10-10.json \
$PROBLEM_DIR/week_2019-10-11.json:$PROBLEM_DIR/week_2019-10-12.json:$PROBLEM_DIR/week_2019-10-13.json:$PROBLEM_DIR/week_2019-10-14.json:$PROBLEM_DIR/week_2019-10-15.json \
$PROBLEM_DIR/week_2019-10-16.json:$PROBLEM_DIR/week_2019-10-17.json:$PROBLEM_DIR/week_2019-10-18.json:$PROBLEM_DIR/week_2019-10-19.json:$PROBLEM_DIR/week_2019-10-20.json \
$PROBLEM_DIR/week_2019-10-21.json:$PROBLEM_DIR/week_2019-10-22.json:$PROBLEM_DIR/week_2019-10-23.json:$PROBLEM_DIR/week_2019-10-24.json:$PROBLEM_DIR/week_2019-10-25.json \
$PROBLEM_DIR/week_2019-10-26.json:$PROBLEM_DIR/week_2019-10-27.json:$PROBLEM_DIR/week_2019-10-28.json:$PROBLEM_DIR/week_2019-10-29.json:$PROBLEM_DIR/week_2019-10-30.json \
$PROBLEM_DIR/week_2019-10-26.json:$PROBLEM_DIR/week_2019-10-27.json:$PROBLEM_DIR/week_2019-10-28.json:$PROBLEM_DIR/week_2019-10-29.json:$PROBLEM_DIR/week_2019-10-30.json)

for  problem in "${PROBLEMS[@]}"; do
echo $problem;
done

#$SOLVER
#--problems=
#--gap-limit=0.05
#--interval-step=00:01:00
#--output=solution_multiple_steps.json