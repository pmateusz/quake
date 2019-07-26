import quake.city

import quake.weather.time_period
import quake.weather.metadata


class Solution:

    def __init__(self, json_object):
        self.__json_object = json_object

    def observations(self, station: quake.city.City):
        all_observations = [observations for station_name, observations in self.__json_object['observations'] if station_name == station.name]

        if not all_observations:
            return []

        assert len(all_observations) == 1

        return [quake.weather.time_period.TimePeriod.from_json(json_object) for json_object in all_observations[0]]

    @property
    def gap(self):
        return self.__metadata.get(quake.weather.metadata.GAP, None)

    @property
    def gap_limit(self):
        return self.__metadata.get(quake.weather.metadata.GAP_LIMIT, None)

    @property
    def interval_step(self):
        return self.__metadata.get(quake.weather.metadata.INTERVAL_STEP, None)

    @property
    def observation_period(self):
        return self.__metadata[quake.weather.metadata.OBSERVATION_PERIOD]

    @property
    def scenario_generator(self):
        return self.__metadata[quake.weather.metadata.SCENARIO_GENERATOR]

    @property
    def scenarios_number(self):
        return self.__metadata.get(quake.weather.metadata.SCENARIOS_NUMBER, None)

    @property
    def solution_type(self):
        return self.__metadata.get(quake.weather.metadata.SOLUTION_TYPE, None)

    @property
    def solution_method(self):
        return self.__metadata.get(quake.weather.metadata.SOLUTION_METHOD, None)

    @property
    def stations(self):
        return [quake.city.City.from_name(station_name) for station_name, observations in self.__json_object['observations']]

    @property
    def time_limit(self):
        return self.__metadata.get(quake.weather.metadata.TIME_LIMIT, None)

    @property
    def target_traffic_index(self):
        return self.__metadata.get(quake.weather.metadata.TARGET_TRAFFIC_INDEX, None)

    @property
    def __metadata(self):
        return quake.weather.metadata.from_json(self.__json_object['metadata'])
