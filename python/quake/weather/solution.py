import datetime
from typing import List, Dict, Any

import quake.city
import quake.weather.metadata
import quake.weather.time_period


class Solution:

    def __init__(self, json_object: Dict[str, Any]):
        self.__json_object = json_object

    def observations(self, station: quake.city.City) -> List[quake.weather.time_period.TimePeriod]:
        all_observations = [observations for station_name, observations in self.__json_object['observations'] if station_name == station.name]

        if not all_observations:
            return []

        assert len(all_observations) == 1

        return [quake.weather.time_period.TimePeriod.from_json(json_object) for json_object in all_observations[0]]

    @property
    def gap(self) -> float:
        return self.__metadata.get(quake.weather.metadata.GAP, None)

    @property
    def gap_limit(self) -> float:
        return self.__metadata.get(quake.weather.metadata.GAP_LIMIT, None)

    @property
    def interval_step(self) -> datetime.timedelta:
        return self.__metadata.get(quake.weather.metadata.INTERVAL_STEP, None)

    @property
    def observation_period(self) -> quake.weather.time_period.TimePeriod:
        return self.__metadata[quake.weather.metadata.OBSERVATION_PERIOD]

    @property
    def scenario_generator(self) -> str:
        return self.__metadata[quake.weather.metadata.SCENARIO_GENERATOR]

    @property
    def scenarios_number(self) -> int:
        return self.__metadata.get(quake.weather.metadata.SCENARIOS_NUMBER, None)

    @property
    def solution_type(self) -> str:
        return self.__metadata.get(quake.weather.metadata.SOLUTION_TYPE, None)

    @property
    def solution_method(self) -> str:
        return self.__metadata.get(quake.weather.metadata.SOLUTION_METHOD, None)

    @property
    def stations(self) -> List[quake.city.City]:
        return [quake.city.City.from_name(station_name) for station_name, observations in self.__json_object['observations']]

    @property
    def time_limit(self) -> datetime.timedelta:
        return self.__metadata.get(quake.weather.metadata.TIME_LIMIT, None)

    @property
    def target_traffic_index(self) -> float:
        return self.__metadata.get(quake.weather.metadata.TARGET_TRAFFIC_INDEX, None)

    @property
    def epsilon(self) -> float:
        return self.__metadata.get(quake.weather.metadata.EPSILON, None)

    @property
    def __metadata(self) -> Dict[str, Any]:
        return quake.weather.metadata.from_json(self.__json_object['metadata'])
