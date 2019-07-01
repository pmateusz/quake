# QUAKE

## Development

```shell
git clone git@github.com:pmateusz/quake.git
mkdir build && cd build
cmake ..
make --jobs 4
```

## Data Sources

Astronomical data about sunrise and sunset were downloaded from [Her Majesty's Nautical Almanac Office](http://astro.ukho.gov.uk/).

## Generate MiniZinc files
```shell
./quake-main --command=minizinc --date=2019-03-29 --time_step=1 --output=2s_1sec_30switch_int.dzn --stations=London,Glasgow --convert-float-to-int
./quake-main --command=minizinc --date=2019-03-29 --time_step=1 --output=all_1sec_30switch_int.dzn --convert-float-to-int
```

## Generate elevation data
```shell
./quake-main --command=elevation --output=elevation.csv --date=2019-03-29 --duration=172800
```

## Run computations
```shell
./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/all_1sec_30switch_int.dzn --output=all_1sec_30switch_int_9-18.json --min-jobs=9 --max-jobs=18 --job_step=1 --failure-scaling-factor=300 --repeats=1 --print-solutions=false
```

## Computational Results
```shell
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/2s_1sec_0switch_int.dzn --output=2s_1sec_0switch_int_2-16.json --min-jobs=2 --max-jobs=16 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/2s_1sec_15switch_int.dzn --output=2s_1sec_15switch_int_2-16.json --min-jobs=2 --max-jobs=16 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/2s_1sec_30switch_int.dzn --output=2s_1sec_30switch_int_2-16.json --min-jobs=2 --max-jobs=16 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/2s_1sec_45switch_int.dzn --output=2s_1sec_45switch_int_2-16.json --min-jobs=2 --max-jobs=16 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/2s_1sec_60switch_int.dzn --output=2s_1sec_60switch_int_2-16.json --min-jobs=2 --max-jobs=16 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false

% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/all_1sec_0switch_int.dzn --output=all_1sec_0switch_int_9-36.json --min-jobs=9 --max-jobs=36 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/all_1sec_15switch_int.dzn --output=all_1sec_15switch_int_9-36.json --min-jobs=9 --max-jobs=36 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/all_1sec_30switch_int.dzn --output=all_1sec_30switch_int_9-36.json --min-jobs=9 --max-jobs=36 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/all_1sec_45switch_int.dzn --output=all_1sec_45switch_int_9-36.json --min-jobs=9 --max-jobs=36 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false
% ./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/all_1sec_60switch_int.dzn --output=all_1sec_60switch_int_9-36.json --min-jobs=9 --max-jobs=36 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=false

./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/all_1week_2019-03-1.dzn --output=all_1week_30switch.json --min-jobs=63 --max-jobs=63 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=true
./quake-main --command=solve-cp --input=/home/pmateusz/dev/quake/data/minizinc/2s_1week_2019-03-1.dzn --output=2s_1week_30switch.json --min-jobs=14 --max-jobs=14 --job_step=1 --failure-scaling-factor=300 --repeats=4 --print-solutions=true

./python/quake.py solution ./data/solution/all_1sec_30switch_int_9-36.json ./data/minizinc/all_1sec_30switch_int.dzn ./data/elevation/elevation_2019_03_29-30.csv ./data/key_rate/data_025eff.ods --output=solution_all_1sec_30switch
./python/quake.py solution ./data/solution/all_1sec_30switch_int_9-36.json ./data/minizinc/all_1sec_30switch_int.dzn ./data/elevation/elevation_2019_03_29-30.csv ./data/key_rate/data_025eff.ods --min-jobs=18 --output=solution_all_1sec_30switch_more_jobs
```

## Troubleshooting
1. During build make asks for username and password to a GitHub account

    Ensure SSH Agent is running. The following command should print one process called ssh-agent.
    
    ```shell
   pmateusz@debian:~/dev/quake/build$ ps -p $SSH_AGENT_PID 
     PID TTY          TIME CMD
     788 ?        00:00:00 ssh-agent
   ```

# Required Libraries
libcurl4
tzdata

