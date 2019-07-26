import copy
import datetime

import numpy as np
import pandas

import quake.city
import quake.cloud_cover
import quake.weather.time_period


class Problem:
    FORECASTS_KEY = 'forecasts'

    def __init__(self, json_object):
        self.__json_object = json_object
        self.__communication_frames = self.__get_communication_frames()

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

    def get_scenario(self, scenario_name):
        forecasts_json = self.__json_object['forecasts']
        forecast_json = forecasts_json[scenario_name]

        index_json = forecast_json['index']
        time_axis = [datetime.datetime.strptime(time_string, '%Y-%b-%d %H:%M:%S') for time_string in index_json]

        index = {}
        stations_cloud_cover_json = forecast_json['stations']
        for station_cloud_cover_json in stations_cloud_cover_json:
            station_name = station_cloud_cover_json['station']
            station = quake.city.City.from_name(station_name)
            cloud_cover_axis = station_cloud_cover_json['cloud_cover']

            assert len(cloud_cover_axis) == len(index_json)
            index[station] = time_axis, copy.copy(cloud_cover_axis)

        return quake.cloud_cover.CloudCoverIndex(index)

    def get_transferred_keys(self, station: quake.city.City, observation_period: quake.weather.time_period.TimePeriod, scenario):
        keys_transferred = 0.0
        for communication_frame in self.__communication_frames[station]:

            frame_period = quake.weather.time_period.TimePeriod(communication_frame.index.min(),  # time period is right open
                                                                communication_frame.index.max() + datetime.timedelta(seconds=1))
            if frame_period.is_after(observation_period):
                break

            if frame_period.is_before(observation_period):
                continue

            overlap_period = frame_period.intersect(observation_period)

            # last time point of the time period is not included
            observation_end_adjusted = observation_period.end - datetime.timedelta(seconds=1)
            overlap_frame = communication_frame[observation_period.begin:observation_end_adjusted]

            if overlap_frame.empty:
                continue

            overlap_frame = overlap_frame.copy()
            overlap_frame['cloud_cover'] = list(map(lambda time: float(scenario(station, time)), overlap_frame.index.to_pydatetime()))

            cloud_cover_coefficients = np.ones(len(overlap_frame)) - overlap_frame['cloud_cover'].values / 100.0
            overlap_frame['effective_transfer'] = np.multiply(overlap_frame['key_rate'].values, cloud_cover_coefficients)
            # overlap_frame['effective_transfer'] = overlap_frame['key_rate'].values # no weather impact

            keys_transferred_locally = overlap_frame['effective_transfer'].sum()
            keys_transferred += keys_transferred_locally
        return keys_transferred

    def transfer_share(self, station: quake.city.City) -> float:
        transfer_shares = self.__get_transfer_shares()
        return transfer_shares[station]

    def __get_transfer_shares(self):
        return {quake.city.City.from_name(station_json['station']): station_json['transfer_share'] for station_json in self.__json_object['stations']}

    def __get_communication_frames(self):
        index = {}

        def __period_to_series(time_period: quake.weather.time_period.TimePeriod, time_step: datetime.timedelta):
            series = []
            current_time = time_period.begin
            while current_time < time_period.end:
                series.append(current_time)
                current_time += time_step
            return series

        for station_json in self.__json_object['stations']:
            station_name = station_json['station']
            station = quake.city.City.from_name(station_name)

            communication_frames = []
            for communication_window_json in station_json['communication_windows']:
                elevation_json = communication_window_json['elevation']
                key_rate_json = communication_window_json['key_rate']
                period_json = communication_window_json['period']
                assert len(elevation_json) == len(key_rate_json)

                period = quake.weather.time_period.TimePeriod.from_json(period_json)
                assert len(key_rate_json) == period.length.total_seconds()

                time_index = __period_to_series(period, datetime.timedelta(seconds=1))
                assert len(key_rate_json) == len(time_index)

                communication_frame = pandas.DataFrame(index=time_index)
                communication_frame['elevation'] = elevation_json
                communication_frame['key_rate'] = key_rate_json
                communication_frames.append(communication_frame)

            assert station not in index
            index[station] = communication_frames

        return index

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
