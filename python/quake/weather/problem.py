import copy
import datetime
import json
import operator
import typing

import numpy as np
import pandas

import quake.city
import quake.cloud_cover
import quake.weather.metadata
import quake.weather.scenarios
import quake.weather.time_period


class Problem:
    FORECASTS_KEY = 'forecasts'
    DECIMAL_PLACES = 3

    def __init__(self, json_object):
        self.__json_object = json_object
        self.__communication_frames = self.__get_communication_frames()

    def add_forecast(self, name, frame):
        if self.FORECASTS_KEY not in self.__json_object:
            self.__json_object[self.FORECASTS_KEY] = {}
        self.__json_object[self.FORECASTS_KEY][name] = self.__frame_to_dict(frame)

    def trim_observation_period(self, new_observation_period: quake.weather.time_period.TimePeriod):
        self.set_metadata(quake.weather.metadata.OBSERVATION_PERIOD, new_observation_period)

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
        metadata = quake.weather.metadata.from_json(self.__json_object['metadata'])
        metadata[key] = value
        self.__json_object['metadata'] = quake.weather.metadata.to_json(metadata)

    def set_var_model(self, var_model):
        var_model_to_use = []
        for station in var_model:
            station_object = dict()
            for item in var_model[station].items():
                if isinstance(item[1], dict):
                    converted_values = {}
                    for key, value in item[1].items():
                        if isinstance(value, float):
                            converted_values[key] = round(value, self.DECIMAL_PLACES)
                        else:
                            converted_values[key] = value
                    station_object[item[0]] = converted_values
                else:
                    if isinstance(item[1], list):
                        station_object[item[0]] = [round(value, self.DECIMAL_PLACES) for value in item[1]]
                    else:
                        station_object[item[0]] = item[1]
            var_model_to_use.append([station, station_object])

        self.__json_object['var_model'] = var_model_to_use

    def set_mean_variance(self, mean_variance_result: quake.weather.scenarios.MeanVarianceResult):
        model_to_use = dict()
        model_to_use['confidence'] = mean_variance_result.confidence_interval
        model_to_use['index'] = [str(value) for value in mean_variance_result.mean.index.values]

        mean_frame = mean_variance_result.mean.round(self.DECIMAL_PLACES)
        mean_lower_frame = mean_variance_result.mean_lower.round(self.DECIMAL_PLACES)
        mean_upper_frame = mean_variance_result.mean_upper.round(self.DECIMAL_PLACES)
        variance_frame = mean_variance_result.variance.round(self.DECIMAL_PLACES)
        variance_lower_frame = mean_variance_result.variance_lower.round(self.DECIMAL_PLACES)
        variance_upper_frame = mean_variance_result.variance_upper.round(self.DECIMAL_PLACES)

        for city in mean_variance_result.mean.columns:
            if not isinstance(city, quake.city.City):
                continue

            city_model = dict()
            city_model['mean'] = mean_frame[city].tolist()
            city_model['mean_lower'] = mean_lower_frame[city].tolist()
            city_model['mean_upper'] = mean_upper_frame[city].tolist()
            city_model['variance'] = variance_frame[city].tolist()
            city_model['variance_lower'] = variance_lower_frame[city].tolist()
            city_model['variance_upper'] = variance_upper_frame[city].tolist()

            model_to_use[city.name] = city_model

        self.__json_object['mean_variance_model'] = model_to_use

    def get_key_rate_frame(self, scenario_name=None) -> pandas.DataFrame:

        forecast = None
        if scenario_name is not None:
            forecast = self.get_scenario(scenario_name=scenario_name)

        frames = []
        for station in self.stations:
            for frame in self.__communication_frames[station]:
                key_rate_frame = frame['key_rate'].to_frame()

                if forecast:
                    cloud_cover = np.array(list(map(lambda date_time: float(forecast(station, date_time)), key_rate_frame.index.to_pydatetime())))
                    cloud_cover_coefficients = np.ones(len(key_rate_frame)) - cloud_cover / 100.0
                    key_rate_frame['effective_key_rate'] = np.multiply(key_rate_frame['key_rate'].values, cloud_cover_coefficients)
                else:
                    key_rate_frame['effective_key_rate'] = key_rate_frame['key_rate']

                key_rate_frame['city'] = station
                key_rate_frame['date_time'] = key_rate_frame.index
                key_rate_frame.reset_index(drop=True, inplace=True)
                frames.append(key_rate_frame)
        master_frame = pandas.concat(frames)
        master_pivot_frame = pandas.pivot_table(master_frame, columns=['city'], values=['effective_key_rate'], index=['date_time'])
        master_pivot_frame.columns = master_pivot_frame.columns.droplevel()
        master_pivot_frame.fillna(value=0.0, inplace=True)
        master_pivot_frame = master_pivot_frame / 2.0  # correction from 128 keys to 256 keys
        return master_pivot_frame

    def get_cloud_cover_frame(self, scenario='real') -> pandas.DataFrame:
        forecast = self.get_scenario(scenario)
        data = []
        for station in self.stations:
            for period in self.get_communication_windows(station):
                for delay in range(0, int(period.length.total_seconds())):
                    date_time = period.begin + datetime.timedelta(seconds=delay)
                    data.append({'station': station, 'date_time': date_time, 'cloud_cover': forecast(station, date_time)})
        frame = pandas.DataFrame(data=data)
        pivot_frame = frame.pivot_table(index=['date_time'], columns=['station'], values=['cloud_cover'])
        pivot_frame.columns = pivot_frame.columns.get_level_values(1)
        return pivot_frame

    def get_scenario(self, scenario_name) -> quake.cloud_cover.CloudCoverIndex:
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

            # overlap_period = frame_period.intersect(observation_period)

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

    def get_communication_windows(self, station: quake.city.City) -> typing.List[quake.weather.time_period.TimePeriod]:
        windows = []
        for station_json in self.__json_object['stations']:
            local_station_name = station_json['station']
            local_station = quake.city.City.from_name(local_station_name)

            if local_station != station:
                continue

            for communication_window_json in station_json['communication_windows']:
                period_json = communication_window_json['period']
                window = quake.weather.time_period.TimePeriod.from_json(period_json)
                windows.append(window)
        return windows

    @property
    def json_object(self):
        return self.__json_object

    @property
    def stations(self):
        cities = [quake.city.from_name(station_data['station']) for station_data in self.__json_object['stations']]
        cities.sort(key=operator.attrgetter('latitude'), reverse=True)
        return cities

    @property
    def observation_period(self) -> quake.weather.time_period.TimePeriod:
        metadata = quake.weather.metadata.from_json(self.__json_object['metadata'])
        return metadata[quake.weather.metadata.OBSERVATION_PERIOD]

    @property
    def scenario_generator(self):
        metadata = quake.weather.metadata.from_json(self.__json_object['metadata'])
        return metadata[quake.weather.metadata.SCENARIO_GENERATOR]

    @staticmethod
    def read_json(file_path: str) -> 'Problem':
        with open(file_path, 'r') as input_stream:
            json_body = json.load(input_stream)
            return Problem(json_body)

    @staticmethod
    def __frame_to_dict(frame):
        station_data = []
        cities = frame.columns.values
        for city in cities:
            cloud_cover_data = frame[city].values.tolist()
            station_data.append({'station': city.name, 'cloud_cover': cloud_cover_data})

        index_values = [value.strftime(quake.weather.time_period.TimePeriod.DATETIME_FORMAT) for value in frame.index]
        return {'index': index_values, 'stations': station_data}
