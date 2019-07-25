import copy

import quake.weather.time_period


class Problem:
    FORECASTS_KEY = 'forecasts'

    def __init__(self, json_object):
        self.__json_object = json_object

    def add_forecast(self, name, frame):
        if self.FORECASTS_KEY not in self.__json_object:
            self.__json_object[self.FORECASTS_KEY] = {}
        self.__json_object[self.FORECASTS_KEY][name] = self.__frame_to_dict(frame)

    def trim_observation_period(self, new_observation_period: quake.weather.time_period.TimePeriod):
        metadata = self.__get_metadata()
        metadata['observation_period'] = new_observation_period.to_json()
        self.__set_metadata(metadata)

        updated_stations = []
        for station_dict in self.__json_object['stations']:
            updated_communication_windows = []
            for communication_window in station_dict['communication_windows']:
                window_period = quake.weather.time_period.TimePeriod.from_json(communication_window['period'])
                if window_period.is_after(new_observation_period) or window_period.is_before(new_observation_period):
                    continue
                updated_communication_windows.append(copy.deepcopy(communication_window))
            updated_station_dict = dict()
            updated_station_dict['communication_windows'] = updated_communication_windows
            for key in station_dict:
                if key == 'communication_windows':
                    continue
                updated_station_dict[key] = copy.deepcopy(station_dict[key])
            updated_stations.append(updated_station_dict)
        self.__json_object['stations'] = updated_stations

    def set_metadata(self, key, value):
        metadata = self.__get_metadata()
        metadata[key] = value
        self.__set_metadata(metadata)

    def __get_metadata(self):
        return {pair[0]: pair[1] for pair in self.__json_object['metadata']}

    def __set_metadata(self, metadata):
        self.__json_object['metadata'] = [[key, value] for key, value in metadata.items()]

    @property
    def json_object(self):
        return self.__json_object

    @property
    def observation_period(self):
        metadata = self.__get_metadata()
        return quake.weather.time_period.TimePeriod.from_json(metadata['observation_period'])

    @staticmethod
    def __frame_to_dict(frame):
        station_data = []
        cities = frame.columns.values
        for city in cities:
            cloud_cover_data = frame[city].values.tolist()
            station_data.append({'station': city.name, 'cloud_cover': cloud_cover_data})

        index_values = [value.strftime(quake.weather.time_period.TimePeriod.DATETIME_FORMAT) for value in frame.index]
        return {'index': index_values, 'stations': station_data}
