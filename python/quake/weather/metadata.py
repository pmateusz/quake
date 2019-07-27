import datetime

import quake.weather.time_period

GAP = 'gap'
GAP_LIMIT = 'gap_limit'
INTERVAL_STEP = 'interval_step'
OBSERVATION_PERIOD = 'observation_period'
SCENARIO_GENERATOR = 'scenario_generator'
SCENARIOS_NUMBER = 'scenarios_number'
SOLUTION_METHOD = 'solution_method'
SOLUTION_TYPE = 'solution_type'
SWITCH_DURATION = 'switch_duration'
TIME_LIMIT = 'time_limit'
TARGET_TRAFFIC_INDEX = 'target_traffic_index'
EPSILON = 'epsilon'


def from_json(json_metadata: dict) -> dict:
    def __parse_timedelta(string_value):
        time = datetime.datetime.strptime(string_value, '%H:%M:%S')
        return datetime.timedelta(hours=time.hour, minutes=time.minute, seconds=time.second)

    metadata = {}
    for key, value in json_metadata:
        value_to_use = value
        if key == OBSERVATION_PERIOD:
            value_to_use = quake.weather.time_period.TimePeriod.from_json(value)
        elif key == SWITCH_DURATION or key == INTERVAL_STEP:
            value_to_use = __parse_timedelta(value)
        metadata[key] = value_to_use
    return metadata


def to_json(metadata: dict) -> dict:
    def __format_timedelta(value: datetime.timedelta):
        remaining_time = int(value.total_seconds())

        hours = remaining_time // 3600
        remaining_time -= hours * 3600
        assert remaining_time >= 0

        minutes = remaining_time // 60
        remaining_time -= minutes * 60
        assert remaining_time >= 0

        seconds = remaining_time
        return '{0:2d}:{1:2d}:{2:2d}'.format(hours, minutes, seconds)

    metadata_json = {}
    for key, value in metadata.items():
        value_to_use = value
        if key == OBSERVATION_PERIOD:
            value_to_use = quake.weather.time_period.TimePeriod.to_json(value)
        elif key == SWITCH_DURATION or key == INTERVAL_STEP:
            value_to_use = __format_timedelta(value)
        metadata[key] = value_to_use
    return metadata_json
