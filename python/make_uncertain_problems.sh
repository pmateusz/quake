#!/usr/bin/env bash

/home/pmateusz/dev/quake/python/weather-tools.py generate --problem-prefix=week --from=2019-11-25 --to=2019-12-01 \
--time-horizon="5 days" --time-step="1 day" --initial-epoch="2019-01-01 00:00:00" \
--num-scenarios=256 --scenario-generator=past_error_replication --stations=London,Manchester,Glasgow,Bristol,Birmingham