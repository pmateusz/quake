#!/usr/bin/env bash
RUN_SCRIPT=/home/pmateusz/dev/quake/python/year-tools.py
PLOT_SCRIPT=/home/pmateusz/dev/quake/python/plot-quake.py

function solve_all_make_plots {
problem_directory=$1
pushd $problem_directory
mkdir -p solutions
pushd solutions
$RUN_SCRIPT run --problem-dir=$problem_directory --solution-prefix=solution --gap-limit=0.005 --time-step=00:00:05
#$PLOT_SCRIPT service-level --data-dir=$problem_directory --solution-dir=$problem_directory/solutions
#$PLOT_SCRIPT network-traffic --data-dir=$problem_directory --solution-dir=$problem_directory/solutions
#$PLOT_SCRIPT long-term-performance --data-dir=$problem_directory --solution-dir=$problem_directory/solutions
#$PLOT_SCRIPT week-performance $problem_directory/year_2013.json $problem_directory/solutions
#$PLOT_SCRIPT aggregate --data-dir=$problem_directory
#$PLOT_SCRIPT communication-window --data-dir=$problem_directory
#$PLOT_SCRIPT print-service-levels
popd
popd
}

#solve_all_make_plots /home/pmateusz/dev/quake/current_review/rc_alt560_inc97.631754

#+------------+----------+-------------+------------+--------------+-------------+
#| station    |   weight |   local_100 |   local_99 |   global_100 |   global_99 |
#|------------+----------+-------------+------------+--------------+-------------|
#| Thurso     |    0.01  |         168 |        178 |            0 |         135 |
#| Glasgow    |    0.111 |        1724 |       1780 |            0 |        1504 |
#| Belfast    |    0.079 |         410 |       1278 |            0 |        1070 |
#| York       |    0.023 |           0 |        390 |            0 |         311 |
#| Manchester |    0.111 |        1737 |       1816 |            0 |        1504 |
#| Birmingham |    0.158 |        2481 |       2561 |            0 |        2140 |
#| Cambridge  |    0.028 |         469 |        502 |            0 |         379 |
#| Ipswich    |    0.02  |           0 |        339 |            0 |         271 |
#| London     |    0.363 |        4920 |       6053 |            0 |        4918 |
#| Bristol    |    0.096 |          17 |       1554 |            0 |        1300 |
#+------------+----------+-------------+------------+--------------+-------------+


#solve_all_make_plots /home/pmateusz/dev/quake/current_review/rc_alt566.899126024325710_inc97.631754

#+------------+----------+-------------+------------+--------------+-------------+
#| station    |   weight |   local_100 |   local_99 |   global_100 |   global_99 |
#|------------+----------+-------------+------------+--------------+-------------|
#| Thurso     |    0.01  |         238 |        242 |            0 |         207 |
#| Glasgow    |    0.111 |        2313 |       2391 |            0 |        2307 |
#| Belfast    |    0.079 |        1655 |       1731 |            0 |        1642 |
#| York       |    0.023 |           1 |        524 |            0 |         478 |
#| Manchester |    0.111 |        2273 |       2415 |            0 |        2307 |
#| Birmingham |    0.158 |        3284 |       3484 |            0 |        3284 |
#| Cambridge  |    0.028 |         675 |        679 |            0 |         582 |
#| Ipswich    |    0.02  |          26 |        483 |            0 |         415 |
#| London     |    0.363 |        6774 |       8441 |            0 |        7546 |
#| Bristol    |    0.096 |           0 |       2243 |            0 |        1995 |
#+------------+----------+-------------+------------+--------------+-------------+

for problem_directory in $(ls /home/pmateusz/dev/quake/current_review)
do
solve_all_make_plots /home/pmateusz/dev/quake/current_review/$problem_directory
done