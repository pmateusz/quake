import quake.city

import quake.weather.time_period


class Solution:

    def __init__(self, json_object):
        self.__json_object = json_object

    def Observations(self, station: quake.city.City):
        all_observations = [observations for station_name, observations in self.__json_object['observations'] if station_name == station.name]

        if not all_observations:
            return []

        assert len(all_observations) == 1

        return [quake.weather.time_period.TimePeriod.from_json(json_object) for json_object in all_observations[0]]

    @property
    def Stations(self):
        return [quake.city.City.from_name(station_name) for station_name, observations in self.__json_object['observations']]
