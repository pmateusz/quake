#!/usr/bin/env bash

pushd /home/pmateusz/dev/quake/python

weather-tools.py plot-forecast --from=2019-07-15  --to=2019-07-20 --output-prefix=forecast_accuracy
weather-tools.py plot-coregionalization --forecast-start-time=2019-07-15 --observation-start-time=2019-07-1 --output-prefix=coregionalization

popd