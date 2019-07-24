#!/usr/bin/env python3
#
# Copyright 2019 Mateusz Polnik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the folowing conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import collections
import concurrent.futures
import concurrent.futures.process
import copy
import datetime
import json
import logging
import os
import subprocess
import warnings

import GPy
import matplotlib.colors
import matplotlib.dates
import matplotlib.offsetbox
import matplotlib.pyplot
import matplotlib.ticker
import numpy
import pandas
import quake.city
import tabulate
import tqdm

# TODO: read how others forecast weather
# TODO: study gaussian process, multi-output gaussian process, non-parametric gaussian regression
# TODO: compute series of errors in all locations

BUILD_CACHE_COMMAND = 'build-cache'
PLOT_COMMAND = 'plot-cloud-cover'
COVARIANCE_COMMAND = 'compute-covariance'
PLOT_FORECAST_COMMAND = 'plot-forecast'
PLOT_COREGIONALIZATION_COMMAND = 'plot-coregionalization'
PLOT_GENERATED_SCENARIOS_COMMAND = 'plot-generated-scenarios'
VAR_COMMAND = 'compute-var'
EXTEND_COMMAND = 'extend'
GENERATE_COMMAND = 'generate'
TRAIN_GP_COMMAND = 'train-gp'

BBOX_STYLE = {'boxstyle': 'square,pad=0.0', 'lw': 0, 'fc': 'w', 'alpha': 0.8}


def load_data_frame(file_path):
    with open(file_path, 'r') as file_stream:
        master_record = json.load(file_stream)
        data = []
        for leaf_record in master_record:
            city = quake.city.from_key(leaf_record['city']['id'])
            count = int(leaf_record['cnt'])
            status_code = int(leaf_record['cod'])
            assert status_code == 200

            local_records = 0
            for record in leaf_record['list']:
                time = datetime.datetime.strptime(record['dt_txt'], '%Y-%m-%d %H:%M:%S')
                cloud_cover = record['clouds']['all']
                description = record['weather'][0]['description']
                data.append((city, time, cloud_cover, description))
                local_records += 1
            assert local_records == count

        data_frame = pandas.DataFrame(columns=['City', 'DateTime', 'CloudCover', 'Description'], data=data)
        min_time = data_frame['DateTime'].min()
        data_frame['Delay'] = data_frame['DateTime'] - min_time
        return data_frame


class WeatherCache:
    HDF_FILE_TABLE = 'a'
    HDF_FILE_MODE = 'w'
    HDF_FORMAT = 'fixed'
    FORECAST_CACHE_FILE = 'forecast.hdf'
    OBSERVATION_CACHE_FILE = 'observation.hdf'
    ZERO_TIME_DELTA = datetime.timedelta()

    def __init__(self):
        self.__forecast_frame = None
        self.__observation_frame = None
        self.__forecast_length = datetime.timedelta(days=4, hours=21)
        self.__forecast_frequency = datetime.timedelta(hours=3)

    def rebuild(self):

        def load_observation_frame(file_path):
            data_frame = pandas.read_csv(file_path)
            data_frame['date_time'] = data_frame['dt_iso'].apply(
                lambda date_string: datetime.datetime.strptime(date_string, '%Y-%m-%d %H:%M:%S %z %Z'))
            data_frame.drop(columns=['weather_id', 'weather_icon', 'dt', 'dt_iso',
                                     'lat', 'lon',
                                     'rain_today', 'snow_today',
                                     'rain_1h', 'snow_1h',
                                     'rain_3h', 'snow_3h',
                                     'rain_24h', 'snow_24h'], inplace=True)
            data_frame['city_name'] = data_frame['city_id'].apply(quake.city.from_key)
            return data_frame

        resolved_root_directory = os.path.expanduser('~/OneDrive/dev/quake/data/forecasts/')

        file_paths = [os.path.abspath(os.path.join(resolved_root_directory, file_name))
                      for file_name in os.listdir(resolved_root_directory) if file_name.endswith('.json')]

        data_frames = []
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future_to_path = {executor.submit(load_data_frame, file_path): file_path for file_path in file_paths}
                for future in tqdm.tqdm(concurrent.futures.as_completed(future_to_path), total=len(future_to_path)):
                    file_path = future_to_path[future]
                    try:
                        data_frames.append(future.result())
                    except Exception as ex:
                        logging.exception('%s generated error %s', file_path, ex)
        forecast_frame = pandas.concat(data_frames)
        forecast_frame.drop_duplicates(inplace=True)
        forecast_frame = forecast_frame.infer_objects()
        forecast_frame.set_index(['DateTime', 'Delay', 'City'], inplace=True, drop=True)
        forecast_frame.sort_index(0, inplace=True)

        observation_frame = load_observation_frame('/home/pmateusz/dev/quake/data/weather/ground_station_data_set_filled.csv')
        observation_frame = observation_frame.infer_objects()

        self.__forecast_frame = forecast_frame
        self.__observation_frame = observation_frame

    def load(self):
        self.__forecast_frame = pandas.read_hdf(self.FORECAST_CACHE_FILE, self.HDF_FILE_TABLE)
        # temporarily disabled for performance
        # self.__observation_frame = pandas.read_hdf(self.OBSERVATION_CACHE_FILE, self.HDF_FILE_TABLE)

    def save(self):
        self.__forecast_frame.to_hdf(self.FORECAST_CACHE_FILE, self.HDF_FILE_TABLE, mode=self.HDF_FILE_MODE, format=self.HDF_FORMAT)

        observation_frame = self.__observation_frame.copy()
        observation_frame['city_name'] = observation_frame['city_name'].astype(str)
        observation_frame.to_hdf(self.OBSERVATION_CACHE_FILE, self.HDF_FILE_TABLE, mode=self.HDF_FILE_MODE, format=self.HDF_FORMAT)

    def get_forecast_frame(self, start_time, duration):
        end_time = start_time + duration
        first_filter_frame = self.__forecast_frame[start_time:end_time]
        second_filter_frame = first_filter_frame[first_filter_frame.index.get_level_values('DateTime')
                                                 - first_filter_frame.index.get_level_values('Delay') == start_time]
        return self.__pivot_transform(second_filter_frame)

    def get_observation_frame(self, start_time, duration):
        end_time = start_time + duration
        data_frame = self.__forecast_frame.loc[pandas.IndexSlice[start_time:end_time, self.ZERO_TIME_DELTA], :]
        return self.__pivot_transform(data_frame)

    def __get_forecast_frame_or_resolve(self, start_time, duration):
        end_time = start_time + duration

        filter_frame \
            = self.__forecast_frame[(self.__forecast_frame['DateTime'] >= start_time) & (self.__forecast_frame['DateTime'] <= end_time)].copy()
        filter_frame['ForecastDateTime'] = filter_frame['DateTime'] - filter_frame['Delay']
        filter_frame['ForecastDateTimeDiff'] = filter_frame['ForecastDateTime'].apply(lambda value: abs((value - start_time).total_seconds()))

        if filter_frame.empty:
            warnings.warn('No forecast records found in requested period [{0}, {1}]'.format(start_time, end_time))
            return pandas.DataFrame(columns=self.__forecast_frame.columns), []

        min_forecast_date_time_diff = filter_frame['ForecastDateTimeDiff'].min()
        forecast_date_time = filter_frame[filter_frame['ForecastDateTimeDiff'] == min_forecast_date_time_diff]['ForecastDateTime'].iloc[0]

        data_frame = filter_frame[filter_frame['ForecastDateTime'] == forecast_date_time].copy()
        return self.__pivot_transform(data_frame)

    # def get_error_frames(self):
    #     forecast_time_points = {datetime.datetime.combine(pandas.to_datetime(time_point).date(), datetime.time())
    #                             for time_point in self.forecast_frame['DateTime']}
    #     filtered_time_points = list(forecast_time_points)
    #     filtered_time_points.sort()
    #
    #     num_values = int(self.forecast_length.total_seconds() / self.__forecast_frequency.total_seconds()) + 1
    #     max_filled_values = int(0.05 * num_values)
    #
    #     error_frames = []
    #     for time_point in filtered_time_points:
    #         forecast_frame, forecast_filled = self.get_forecast_frame(time_point, self.forecast_length, fill_missing=True)
    #
    #         if len(forecast_filled) > max_filled_values or len(forecast_frame) != num_values:
    #             continue
    #
    #         observation_frame, observation_filled = self.get_observation_frame(time_point, self.forecast_length, fill_missing=True)
    #
    #         if len(observation_filled) > max_filled_values or len(observation_frame) != num_values:
    #             continue
    #
    #         error_frame = forecast_frame - observation_frame
    #
    #         # error frame has no missing values
    #         assert error_frame.isna().sum().max() == 0
    #
    #         error_frames.append(error_frame)
    #     return error_frames

    def get_forecast_time_points(self):
        filter_frame = self.__forecast_frame.loc[pandas.IndexSlice[:, self.ZERO_TIME_DELTA], :]
        time_points = list(filter_frame.index.get_level_values('DateTime').unique().to_pydatetime())
        return time_points
        # forecast_time_points = {datetime.datetime.combine(pandas.to_datetime(time_point).date(), datetime.time()) for time_point in time_points}
        # filtered_time_points = list(forecast_time_points)
        # filtered_time_points.sort()
        # return filtered_time_points

    def get_error_frames(self, time_points):
        result = []

        for time_point in tqdm.tqdm(time_points, desc='Creating Error Frames', leave=False):
            forecast_frame = self.get_forecast_frame(time_point, self.forecast_length)
            observation_frame = self.get_observation_frame(time_point, self.forecast_length)
            error_frame = forecast_frame - observation_frame
            error_frame['Delay'] = error_frame.index - error_frame.index.min()
            error_frame.dropna(inplace=True)
            if error_frame.empty:
                continue
            result.append(error_frame)
        return result

    def get_forecast_error_variance(self, error_frames=None):
        if not error_frames:
            error_frames = self.get_error_frames(self.get_forecast_time_points())
        master_error_frame = pandas.concat(error_frames)
        master_error_frame['DateTime'] = master_error_frame.index
        master_error_frame.reset_index(drop=True, inplace=True)

        city_columns = [column for column in master_error_frame.columns if isinstance(column, quake.city.City)]
        melted_master_error_frame = pandas.melt(master_error_frame, value_vars=city_columns,
                                                var_name='City', value_name='Error', id_vars=['DateTime', 'Delay'])
        var_series = melted_master_error_frame.groupby(['City', 'Delay'])['Error'].var()
        return var_series

    def get_forecast_error_covariance(self, city, error_frames):
        rows = []
        time_steps = [datetime.timedelta(hours=value) for value in range(0,
                                                                         int(self.__forecast_length.total_seconds() / 3600) + 1,
                                                                         int(self.__forecast_frequency.total_seconds() / 3600))]
        for error_frame in error_frames:
            min_time = error_frame.index.min()
            max_time = error_frame.index.max()

            series = error_frame[city]
            series_index_set = set(series.index.to_pydatetime().tolist())
            current_time = min_time
            while current_time < max_time:
                row = []
                for time_step in time_steps:
                    time_index = current_time + time_step
                    if time_index in series_index_set:
                        row.append(series[time_index])
                    else:
                        row.append(numpy.NaN)
                rows.append(row)
                current_time = current_time + self.__forecast_frequency
        master_frame = pandas.DataFrame(data=rows, columns=time_steps)
        covariance = master_frame.cov()
        return covariance.values

    @staticmethod
    def __fill_missing_values(frame):
        freq = pandas.infer_freq(frame.index)

        counter = collections.Counter()
        if freq:
            return frame, []
        index_it = iter(frame.index)
        prev_value = next(index_it, None)
        if prev_value:
            for current_value in index_it:
                time_distance = current_value - prev_value
                counter[time_distance] += 1
                prev_value = current_value

        inferred_feq = counter.most_common(1)[0][0]
        start_index = frame.index.min()
        end_index = frame.index.max()

        missing_values = []
        current_index = start_index
        while current_index < end_index:
            if current_index not in frame.index:
                missing_values.append(current_index)
            current_index += inferred_feq

        if missing_values:
            warnings.warn('Filling missing values: {0}'.format(missing_values))

        data = numpy.full((len(missing_values), len(frame.columns)), numpy.NaN)
        missing_values_frame = pandas.DataFrame(index=missing_values, data=data, columns=frame.columns)
        filled_frame = frame.append(missing_values_frame)
        filled_frame.sort_index(inplace=True)
        filled_frame.fillna(method='ffill', inplace=True)
        return filled_frame, missing_values

    @staticmethod
    def __verify_period(frame, start_time, end_time):
        actual_start_time = frame['DateTime'].min()
        actual_end_time = frame['DateTime'].max()

        if actual_start_time > start_time or actual_end_time < end_time:
            warnings.warn('Available data [{0},{1}] does not cover requested period [{2}, {3}]'
                          .format(actual_start_time, actual_end_time, start_time, end_time))

    @staticmethod
    def __pivot_transform(frame):
        pivot_frame = pandas.pivot_table(frame, columns=['City'], index=['DateTime'], values=['CloudCover'])
        pivot_frame.columns = pivot_frame.columns.droplevel()
        pivot_frame.drop_duplicates(inplace=True)
        pivot_frame.sort_index(inplace=True)
        return pivot_frame

    @property
    def forecast_length(self):
        return self.__forecast_length

    @property
    def forecast_frame(self):
        return self.__forecast_frame

    @property
    def observation_frame(self):
        return self.__observation_frame


class TimePeriod:
    DATETIME_FORMAT: str = '%Y-%b-%d %H:%M:%S'

    def __init__(self, begin_time, end_time):
        self.__begin_time = begin_time
        self.__end_time = end_time

    def is_before(self, other):
        return self.end < other.begin

    def is_after(self, other):
        return self.begin >= other.end

    @property
    def begin(self):
        return self.__begin_time

    @property
    def end(self):
        return self.__end_time

    @property
    def length(self):
        return self.__end_time - self.__begin_time

    @staticmethod
    def from_json(json_object):
        begin_datetime = datetime.datetime.strptime(json_object['begin'], TimePeriod.DATETIME_FORMAT)
        end_datetime = datetime.datetime.strptime(json_object['end'], TimePeriod.DATETIME_FORMAT)
        return TimePeriod(begin_datetime, end_datetime)

    def to_json(self):
        return {'begin': self.__begin_time.strftime(self.DATETIME_FORMAT), 'end': self.__end_time.strftime(self.DATETIME_FORMAT)}


class Problem:
    FORECASTS_KEY = 'forecasts'

    def __init__(self, json_object):
        self.__json_object = json_object

    def add_forecast(self, name, frame):
        if self.FORECASTS_KEY not in self.__json_object:
            self.__json_object[self.FORECASTS_KEY] = {}
        self.__json_object[self.FORECASTS_KEY][name] = self.__frame_to_dict(frame)

    def trim_observation_period(self, new_observation_period: TimePeriod):
        metadata = self.__json_object['metadata']
        metadata['observation_period'] = new_observation_period.to_json()

        updated_stations = []
        for station_dict in self.__json_object['stations']:
            updated_communication_windows = []
            for communication_window in station_dict['communication_windows']:
                window_period = TimePeriod.from_json(communication_window['period'])
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

    @property
    def json_object(self):
        return self.__json_object

    @property
    def observation_period(self):
        metadata = self.__json_object['metadata']
        return TimePeriod.from_json(metadata['observation_period'])

    @staticmethod
    def __frame_to_dict(frame):
        station_data = []
        cities = frame.columns.values
        for city in cities:
            cloud_cover_data = frame[city].values.tolist()
            station_data.append({'station': city.name, 'cloud_cover': cloud_cover_data})

        index_values = [value.strftime(TimePeriod.DATETIME_FORMAT) for value in frame.index]
        return {'index': index_values, 'stations': station_data}


class JSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Problem):
            return obj.json_object
        if isinstance(obj, quake.city.City):
            return obj.name
        return json.JSONEncoder.default(self, obj)


class ParseDateAction(argparse.Action):
    DATE_FORMAT = '%Y-%m-%d'

    def __call__(self, parser: argparse.ArgumentParser, namespace: argparse.Namespace, values, option_string=None):
        date_time_value = datetime.datetime.strptime(values, ParseDateAction.DATE_FORMAT)
        setattr(namespace, self.dest, date_time_value)


class CoregionalizationModel:

    def __init__(self, observation_frame, forecast_frame):
        assert observation_frame.index.max() <= forecast_frame.index.min()

        self.__observation_frame = observation_frame
        self.__forecast_frame = forecast_frame
        self.__model = None
        self.__X = None
        self.__Y = None

    def optimize(self):
        # prepare input
        X, Y = [], []
        observation_start_time = self.__observation_frame.index.min()

        def elapsed_hours(date_time):
            return int((date_time - observation_start_time).total_seconds() / 3600)

        cities = self.__forecast_frame.columns
        for city_index, city in enumerate(cities):
            for time, cloud_cover in self.__observation_frame[city].items():
                X.append([time, elapsed_hours(time), city_index])
                Y.append([cloud_cover])

            for time, cloud_cover in self.__forecast_frame[city].items():
                X.append([time, elapsed_hours(time), city_index])
                Y.append([cloud_cover])

        self.__X = numpy.array(X)
        self.__Y = numpy.array(Y)
        del X, Y

        # plot output data
        matrix_rank = len(cities)
        kernel_1 = GPy.kern.RBF(1, lengthscale=6, ARD=True) + GPy.kern.Linear(1, 1, active_dims=[0], ARD=True) \
                   + GPy.kern.White(1, variance=81) + GPy.kern.Bias(1)
        kernel_2 = GPy.kern.Coregionalize(1, output_dim=len(cities), rank=matrix_rank)

        X_Gpy = self.__X[:, [1, 2]]
        self.__model = GPy.models.GPRegression(X_Gpy, self.__Y, kernel_1 ** kernel_2)
        self.__model.optimize(messages=True, max_iters=32)

        mean, variance = self.__model.predict(X_Gpy, full_cov=True)
        quantiles = self.__model.predict_quantiles(X_Gpy)

        X_cities = numpy.array([cities[city_index] for city_index in self.__X[:, 2]])
        data = numpy.hstack((self.__X[:, [0]], X_cities.reshape(-1, 1), self.__Y.reshape(-1, 1), mean, quantiles[0], quantiles[1]))
        data_frame = pandas.DataFrame(data=data, columns=['DateTime', 'City', 'y', 'yhat', 'yhat_lower', 'yhat_upper'])
        data_frame = data_frame.infer_objects()
        return data_frame

    def samples(self, size):
        test_X = self.__X[self.__X[:, 0] >= self.__forecast_frame.index.min()]
        Y_posterior_samples = self.__model.posterior_samples_f(test_X[:, [1, 2]], full_cov=True, size=size)
        time_index = test_X[:, [0]]

        cities = self.__forecast_frame.columns
        X_cities = numpy.array([cities[city_index] for city_index in test_X[:, 2]])
        X_cities = X_cities.reshape(-1, 1)

        sample_frames = []
        for sample_index in range(size):
            data = numpy.hstack((time_index, X_cities, Y_posterior_samples[:, :, sample_index]))
            data_frame = pandas.DataFrame(data=data, columns=['DateTime', 'City', 'y'])
            data_frame = data_frame.infer_objects()
            sample_frames.append(data_frame)
        return sample_frames


class PastErrorsModel:
    DATE_TIME_FREQ = datetime.timedelta(hours=3)

    def __init__(self, weather_cache, forecast_frame):
        self.__weather_cache = weather_cache
        self.__forecast_frame = forecast_frame
        self.__error_frames = []

    def optimize(self):
        error_frames = self.__weather_cache.get_error_frames(self.__weather_cache.get_forecast_time_points())
        num_rows_min = self.__forecast_frame.shape[0]
        num_missing_rows_max = int(0.2 * num_rows_min)
        num_cities = len([column for column in self.__forecast_frame.columns if isinstance(column, quake.city.City)])

        # get error frames that do not have many missing entries and cover forecast's period fully
        valid_error_frames = []

        for error_frame in error_frames:
            assert not error_frame.empty

            first_date_time = error_frame.index[0] - error_frame.iloc[0]['Delay']
            last_date_time = error_frame.index.max()

            num_rows_after_filling = int((last_date_time - first_date_time).total_seconds() / self.DATE_TIME_FREQ.total_seconds()) + 1
            if num_rows_after_filling < num_rows_min:
                continue

            num_missing_rows = num_rows_min - error_frame.shape[0]
            if num_missing_rows > num_missing_rows_max:
                continue

            missing_entries = []
            current_date_time = first_date_time
            while current_date_time <= last_date_time:
                if current_date_time not in error_frame.index:
                    missing_entries.append(current_date_time)
                current_date_time += self.DATE_TIME_FREQ

            if not missing_entries:
                valid_error_frames.append(error_frame)
                continue

            # create frame with missing entries
            missing_rows = []
            for entry in missing_entries:
                row = [numpy.NaN] * num_cities
                row.append(entry - first_date_time)
                missing_rows.append(row)

            missing_values_frame = pandas.DataFrame(index=missing_entries, data=missing_rows, columns=error_frame.columns)

            # join frames
            filled_error_frame = error_frame.append(missing_values_frame)
            filled_error_frame.sort_index(inplace=True)
            filled_error_frame.fillna(value=0.0, inplace=True)
            valid_error_frames.append(filled_error_frame)

        self.__error_frames = valid_error_frames

    def samples(self, size):
        draw_with_replacement = size > len(self.__error_frames)
        samples_selected = numpy.random.choice(len(self.__error_frames), replace=draw_with_replacement, size=size)

        # column names for a sample frame are: DateTime, City and y
        sample_frames = []
        for sample_index in samples_selected:
            error_frame = self.__error_frames[sample_index]
            values = self.__forecast_frame.values + error_frame.values[:len(self.__forecast_frame),
                                                    :len(error_frame.columns) - 1]  # trim the last delay column
            rows = []
            for city_index, city in enumerate(self.__forecast_frame.columns):
                for row_index, date_time in enumerate(self.__forecast_frame.index):
                    rows.append([date_time, city, values[row_index, city_index]])
            sample_frame = pandas.DataFrame(columns=['DateTime', 'City', 'y'], data=rows)
            sample_frames.append(sample_frame)
        return sample_frames


class HeteroscedasticIndependentNoiseModel:

    def __init__(self, weather_cache, forecast_frame):
        self.__weather_cache = weather_cache
        self.__forecast_frame = forecast_frame
        self.__step_city_variance = None

    def optimize(self):
        error_frames = self.__weather_cache.get_error_frames(self.__weather_cache.get_forecast_time_points())
        self.__step_city_variance = self.__weather_cache.get_forecast_error_variance(error_frames)

    def samples(self, size):
        sample_frames = [self.__generate_sample_frame() for _ in range(size)]
        return sample_frames

    def __generate_sample_frame(self):
        num_rows = self.__forecast_frame.shape[0]

        rows = []
        for city in self.__forecast_frame.columns:
            V = self.__step_city_variance[city].values
            prediction = self.__forecast_frame[city].values + numpy.random.normal(size=num_rows) * numpy.sqrt(V[:num_rows])
            for time_delta, value in zip(self.__forecast_frame.index, prediction):
                rows.append([city, time_delta, value])
        sample_frame = pandas.DataFrame(data=rows, columns=['City', 'DateTime', 'y'])
        return sample_frame


class HeteroscedasticAutoCorrelatedNoiseModel:

    def __init__(self, weather_cache, forecast_frame):
        self.__weather_cache = weather_cache
        self.__forecast_frame = forecast_frame
        self.__city_covariance = None

    def optimize(self):
        error_frames = self.__weather_cache.get_error_frames(self.__weather_cache.get_forecast_time_points())

        self.__city_covariance = dict()
        for city in tqdm.tqdm(self.__forecast_frame.columns, leave=False, desc='Building covariance matrices'):
            self.__city_covariance[city] = self.__weather_cache.get_forecast_error_covariance(city, error_frames)

    def samples(self, size):
        sample_frames = [self.__generate_sample_frame() for _ in range(size)]
        return sample_frames

    def __generate_sample_frame(self):

        rows = []
        for city in self.__forecast_frame.columns:
            city_covariance = self.__city_covariance[city]
            prediction = self.__forecast_frame[city].values + numpy.random.multivariate_normal(numpy.zeros(city_covariance.shape[0]), city_covariance)
            for time_delta, value in zip(self.__forecast_frame.index, prediction):
                rows.append([city, time_delta, value])
        sample_frame = pandas.DataFrame(data=rows, columns=['City', 'DateTime', 'y'])
        return sample_frame


def parse_args():
    parser = argparse.ArgumentParser()

    sub_parsers = parser.add_subparsers(dest='command')

    build_cache_parser = sub_parsers.add_parser(BUILD_CACHE_COMMAND)

    plot_parser = sub_parsers.add_parser(PLOT_COMMAND)
    plot_parser.add_argument('--from')

    covariance_parser = sub_parsers.add_parser(COVARIANCE_COMMAND)

    extend_parser = sub_parsers.add_parser(EXTEND_COMMAND)
    extend_parser.add_argument('problem_file')
    extend_parser.add_argument('--output')
    extend_parser.add_argument('--num-scenarios', default=0, type=int)

    generate_parser = sub_parsers.add_parser(GENERATE_COMMAND)
    generate_parser.add_argument('--from', action=ParseDateAction)
    generate_parser.add_argument('--to', action=ParseDateAction)
    generate_parser.add_argument('--problem-prefix')
    generate_parser.add_argument('--num-scenarios', default=0, type=int)

    plot_forecast_parser = sub_parsers.add_parser(PLOT_FORECAST_COMMAND)
    plot_forecast_parser.add_argument('--from', action=ParseDateAction)
    plot_forecast_parser.add_argument('--to', action=ParseDateAction)
    plot_forecast_parser.add_argument('--output-prefix')

    compute_var = sub_parsers.add_parser(VAR_COMMAND)

    train_gaussian_process = sub_parsers.add_parser(TRAIN_GP_COMMAND)

    plot_coregionalization_parser = sub_parsers.add_parser(PLOT_COREGIONALIZATION_COMMAND)
    plot_coregionalization_parser.add_argument('--observation-start-time', action=ParseDateAction)
    plot_coregionalization_parser.add_argument('--forecast-start-time', action=ParseDateAction)
    plot_coregionalization_parser.add_argument('--output-prefix')

    plot_generated_scenarios_parser = sub_parsers.add_parser(PLOT_GENERATED_SCENARIOS_COMMAND)
    plot_generated_scenarios_parser.add_argument('--from', action=ParseDateAction)
    plot_generated_scenarios_parser.add_argument('--num-scenarios', default=0, type=int)
    plot_generated_scenarios_parser.add_argument('--output-prefix')

    return parser.parse_args()


def build_cache_command(args):
    weather_cache = WeatherCache()
    weather_cache.rebuild()
    weather_cache.save()


def process_parallel_map(data, function):
    results = []
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = [executor.submit(function, chunk) for chunk in data]
        for future in tqdm.tqdm(futures):
            results.append(future.result())
    return results


def covariance_command(args):
    def save_matrix(matrix, file_name):
        csv_file = file_name + '.csv'
        with open(csv_file, 'w') as output_stream:
            matrix.to_csv(output_stream)
        txt_file = file_name + '.txt'
        with open(txt_file, 'w') as output_stream:
            print(tabulate.tabulate(matrix), file=output_stream)

    weather_cache = WeatherCache()
    weather_cache.load()

    observation_frame = weather_cache.observation_frame

    # compute spatial covariance
    pivot_frame = observation_frame.pivot_table(columns=['city_name'], values=['clouds_all'], index=['date_time'])
    pivot_frame.columns = pivot_frame.columns.droplevel(0)
    spatial_covariance_frame = pivot_frame.cov()
    save_matrix(spatial_covariance_frame, 'spatial_covariance')

    # compute temporal covariance
    time_deltas = []
    delta = datetime.timedelta()
    while delta <= datetime.timedelta(hours=5 * 24):
        time_deltas.append(delta)
        delta += datetime.timedelta(hours=3)

    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
        for city in observation_frame['city_name'].unique():
            city_frame = observation_frame[observation_frame['city_name'] == city].copy()

            reference_dict = dict()
            for row in city_frame.itertuples():
                reference_dict[row.date_time] = row.clouds_all

            data = []
            for date_time in tqdm.tqdm(city_frame.date_time, desc=city.name):
                row = [date_time]
                for time_delta in time_deltas:
                    reference_time = date_time + time_delta
                    cloud_cover = reference_dict[reference_time] if reference_time in reference_dict else numpy.nan
                    row.append(cloud_cover)
                data.append(row)
            cloud_cover_frame = pandas.DataFrame(data=data,
                                                 columns=['DateTime', *(str(delta) for delta in time_deltas)])
            temporal_covariance = cloud_cover_frame.cov()
            save_matrix(temporal_covariance, '{0}_temporal_covariance'.format(city.name))


def compute_vector_auto_regression(args):
    weather_cache = WeatherCache()
    weather_cache.load()

    def pivot_transform(frame):
        pivot_frame = pandas.pivot_table(frame, columns=['City'], index=['DateTime'], values=['CloudCover'])
        pivot_frame.columns = pivot_frame.columns.droplevel()
        pivot_frame.drop_duplicates(inplace=True)
        pivot_frame.sort_index(inplace=True)
        return pivot_frame

    def forecast_sample(frame, date_time):
        local_frame = frame.copy()
        foreacast_date_time_series = local_frame['DateTime'] - local_frame['Delay']
        return pivot_transform(local_frame[foreacast_date_time_series == date_time])

    def observation_sample(frame, date_time):
        max_date_time = date_time + datetime.timedelta(days=4, hours=21)
        local_frame = frame.copy()
        return pivot_transform(local_frame[(local_frame['Delay'] == datetime.timedelta(seconds=0))
                                           & (local_frame['DateTime'] >= date_time)
                                           & (local_frame['DateTime'] <= max_date_time)])

    forecast_frame = weather_cache.forecast_frame.copy()

    time_points = [pandas.to_datetime(value)
                   for value in forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))]['DateTime'].sort_values().unique()]
    locations = forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))]['City'].sort_values().unique()

    for time_point in time_points:
        local_forecast_sample = forecast_sample(forecast_frame, time_point)
        local_observation_sample = observation_sample(forecast_frame, time_point)
        for location in locations:
            residual_series = local_forecast_sample[location] - local_observation_sample[location]
            residual_frame = residual_series.to_frame()
            residual_frame['DateTime'] = pandas.to_datetime(residual_frame.index)
            min_time = residual_frame['DateTime'].min()
            residual_frame['Delay'] = residual_frame['DateTime'] - min_time
            pass

    result_frame = pivot_transform(forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))
                                                  & (forecast_frame['DateTime'] >= datetime.datetime(2019, 6, 20))
                                                  & (forecast_frame['DateTime'] <= datetime.datetime(2019, 7, 12))])

    import statsmodels.tsa.api

    model = statsmodels.tsa.api.VAR(result_frame)
    result = model.fit()
    print(result.summary())

    # start_date_time = datetime.datetime(2019, 6, 20)
    # for days in range(10):
    #     date_time = start_date_time + datetime.timedelta(days=days)
    #     local_forecast_sample = forecast_sample(forecast_frame, date_time)
    #     local_observation_sample = observation_sample(forecast_frame, date_time)
    #
    #     errors = local_forecast_sample['London'] - local_observation_sample['London']
    #     errors.fillna(value=0, inplace=True)

    # fig, ax = matplotlib.pyplot.subplots()
    # ax.plot(local_forecast_sample['London'], label='forecast')
    # ax.plot(local_observation_sample['London'], label='observation')
    # ax.legend()
    # fig.tight_layout()
    # matplotlib.pyplot.savefig('London_{0}.png'.format(date_time.strftime('%Y-%m-%d')))
    # matplotlib.pyplot.close(fig)
    #
    # fig, ax = matplotlib.pyplot.subplots()
    # ax.plot(local_observation_sample['London'] - local_forecast_sample['London'], label='diff')
    # ax.legend()
    # fig.tight_layout()
    # matplotlib.pyplot.savefig('diff_London_{0}.png'.format(date_time.strftime('%Y-%m-%d')))
    # matplotlib.pyplot.close(fig)

    # long_term_frame = forecast_frame[
    #     (forecast_frame['Delay'] == datetime.timedelta(days=2)) & (forecast_frame['DateTime'] > datetime.datetime(2019, 6, 20))]
    #
    # long_term_frame = pivot_transform(long_term_frame)
    # observation_frame = pivot_transform(observation_frame)
    #
    # long_term_frame['London'].plot()
    # observation_frame['London'].plot()
    # (long_term_frame['London'] - observation_frame['London']).plot()

    # three_hour_frame['DateTime'] = three_hour_frame['DateTime'] + three_hour_frame['Delay']

    # observation_frame = pivot_transform(observation_frame)
    # three_hour_frame = pivot_transform(three_hour_frame)
    #
    # observation_frame['Glasgow'].plot()
    # three_hour_frame['Glasgow'].plot()

    # matplotlib.pyplot.show()
    #
    # print('here')

    # blue_print = weather_cache.observation_frame.tail()
    # local_frame = weather_cache.observation_frame[['city_name', 'date_time', 'clouds_all']].copy()
    # pivot_frame = pandas.pivot_table(local_frame, columns=['city_name'], index=['date_time'], values=['clouds_all'])
    # pivot_frame.columns = pivot_frame.columns.droplevel()
    # pivot_frame.drop_duplicates(inplace=True)
    # pivot_frame.sort_index(inplace=True)
    # model = statsmodels.tsa.api.VAR(pivot_frame)
    # model_result = model.fit()
    # print(model_result.summary())
    # forecast_frame = weather_cache.forecast_frame.copy()
    # forecast_frame = forecast_frame[forecast_frame['DateTime'] > datetime.datetime(2019, 6, 1)]

    # observations are about to pass unit root test
    # long-term forecasts pass unit root test without question
    # observation_frame = pivot_transform(.copy())
    # long_term_forecast_frame = pivot_transform(forecast_frame[forecast_frame['Delay'] == datetime.timedelta(days=4, hours=21)].copy())
    # local_forecast_frame = pivot_transform(
    # )

    # five_day_forecast = forecast_frame[forecast_frame['DateTime'] == datetime.datetime(2019, 6, 30, 0, 0)].copy()
    # five_day_forecast['DateTime'] = five_day_forecast['DateTime'] + five_day_forecast['Delay']
    #
    # observation_frame = forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))
    #                                    & (forecast_frame['DateTime'] <= five_day_forecast['DateTime'].max())
    #                                    & (forecast_frame['DateTime'] >=five_day_forecast['DateTime'].min())].copy()
    #
    # five_day_frame = pivot_transform(five_day_forecast)
    # observation_frame = pivot_transform(observation_frame)
    #
    #
    # observation_frame.plot()
    #
    # matplotlib.pyplot.show()

    pass

    # local_data_frame = pivot_transform(local_data_frame)

    # def test_for_unit_root(frame):
    #     for column in frame.columns:
    #         pass
    #         # print(column, statsmodels.tsa.stattools.adfuller(frame[column]))
    #         # print(column, statsmodels.tsa.stattools.kpss(frame[column]))
    #
    # test_for_unit_root(observation_frame)
    # test_for_unit_root(long_term_forecast_frame)
    # test_for_unit_root(local_data_frame)
    #
    # model = statsmodels.tsa.api.VAR(observation_frame)
    # results = model.fit()
    # residuals = results.resid
    #
    # residuals['London'].plot()
    # matplotlib.pyplot.show()
    #
    # print(results.is_stable())
    # print(results.summary())

    # long_term_forecast_frame['London'].plot()
    # matplotlib.pyplot.show()

    # TODO: build var forecast for a weather forecast
    pass


def extend_problem_definition(args):
    problem_file = getattr(args, 'problem_file')
    output_file = getattr(args, 'output')
    num_scenarios = getattr(args, 'num_scenarios')

    with open(problem_file, 'r') as input_stream:
        json_object = json.load(input_stream)
        problem = Problem(json_object)

    weather_cache = WeatherCache()
    weather_cache.load()

    forecast_frame = weather_cache.get_forecast_frame(problem.observation_period.begin, problem.observation_period.length, fill_missing=True)
    real_frame = weather_cache.get_observation_frame(problem.observation_period.begin, problem.observation_period.length, fill_missing=True)

    min_time = max(forecast_frame.index.min(), real_frame.index.min())
    max_time = min(forecast_frame.index.max(), real_frame.index.max())

    if problem.observation_period.begin < min_time or problem.observation_period.end > max_time:
        warnings.warn('Problem observation period is reduced from [{0}, {1}] to [{2}, {3}]'.format(problem.observation_period.begin,
                                                                                                   problem.observation_period.end,
                                                                                                   min_time,
                                                                                                   max_time))

    def enforce_cloud_cover_limits(frame):
        frame_to_use = frame.copy()
        for column in frame.columns:
            frame_to_use[column] = frame_to_use[column].apply(lambda value: max(0.0, min(1.0, value)))
        return frame_to_use

    scenario_frames = []
    if num_scenarios > 0:
        # # coregionalization model
        # observation_length = datetime.timedelta(days=14)
        # observation_frame = weather_cache.get_observation_frame(forecast_frame.index.min() - observation_length, observation_length)
        # model = CoregionalizationModel(observation_frame, forecast_frame)

        # past errors model
        model = PastErrorsModel(weather_cache, forecast_frame)
        model.optimize()

        sample_frames = model.samples(num_scenarios)
        for sample_frame in sample_frames:
            scenario_frame = sample_frame.pivot_table(index='DateTime', columns='City', values='y')
            scenario_frame = enforce_cloud_cover_limits(scenario_frame)
            scenario_frames.append(scenario_frame)

    def trim_to_time_interval(frame):
        return frame[(frame.index >= min_time) & (frame.index <= max_time)]

    updated_problem = copy.deepcopy(problem)
    updated_problem.trim_observation_period(TimePeriod(min_time, max_time))
    updated_problem.add_forecast('forecast', trim_to_time_interval(forecast_frame))
    updated_problem.add_forecast('real', trim_to_time_interval(real_frame))

    for scenario_index, scenario_frame in enumerate(scenario_frames):
        updated_problem.add_forecast('scenario_{0}'.format(scenario_index), trim_to_time_interval(scenario_frame))

    with open(output_file, 'w') as output_file:
        json.dump(updated_problem.json_object, output_file)


def generate_command(args):
    GENERATE_PROGRAM_PATH = '/home/pmateusz/dev/quake/cmake-build-debug/quake-generate'

    problem_prefix_arg = getattr(args, 'problem_prefix')
    from_date_arg = getattr(args, 'from')
    to_date_arg = getattr(args, 'to')
    num_scenarios = getattr(args, 'num_scenarios')

    configurations = []
    current_date = from_date_arg
    while current_date < to_date_arg:
        next_date = current_date + datetime.timedelta(days=5)
        next_date = min(next_date, to_date_arg)

        configurations.append((current_date, next_date, '{0}_{1}.json'.format(problem_prefix_arg, current_date.date())))

        current_date = next_date

    temp_suffix = '_v0'
    for from_date, to_date, problem in configurations:

        temp_problem = problem + temp_suffix
        subprocess.run([GENERATE_PROGRAM_PATH,
                        '--from={0}'.format(from_date.date()),
                        '--to={0}'.format(to_date.date()),
                        '--output={0}'.format(temp_problem)], check=True)

        if not os.path.exists(temp_problem):
            raise Exception('Failed to generate problem {0}'.format(temp_problem))

        args = argparse.Namespace(**{'problem_file': temp_problem, 'output': problem, 'num_scenarios': num_scenarios})
        extend_problem_definition(args)

        if not os.path.exists(problem):
            raise Exception('Failed to generate problem {0}'.format(problem))

        os.remove(temp_problem)


# TODO: plug in
def plot_generated_scenarios(args):
    forecast_time = getattr(args, 'from')
    num_samples = getattr(args, 'num_scenarios')
    output_prefix = getattr(args, 'output_prefix')

    weather_cache = WeatherCache()
    weather_cache.load()
    forecast_frame = weather_cache.get_forecast_frame(forecast_time, weather_cache.forecast_length)

    past_error_model = PastErrorsModel(weather_cache, forecast_frame)
    past_error_model.optimize()

    independent_noise_error_model = HeteroscedasticIndependentNoiseModel(weather_cache, forecast_frame)
    independent_noise_error_model.optimize()

    autocorrelated_noise_error_model = HeteroscedasticAutoCorrelatedNoiseModel(weather_cache, forecast_frame)
    autocorrelated_noise_error_model.optimize()

    coregionalized_observation_length = datetime.timedelta(days=14)
    coregionalized_observation_start = forecast_time - coregionalized_observation_length
    coregionalized_observe = weather_cache.get_observation_frame(coregionalized_observation_start, coregionalized_observation_length)
    coregionalized_model = CoregionalizationModel(coregionalized_observe, forecast_frame)
    coregionalized_model.optimize()

    def domain_filter(values):
        return [max(0, min(100, value)) for value in values]

    models = [past_error_model, independent_noise_error_model, autocorrelated_noise_error_model, coregionalized_model]
    model_labels = ['Past Errors Replication', 'Independent Noise Generation', 'Autocorrelated Noise Genration', 'Coregionalization Model']

    observation_frame = weather_cache.get_observation_frame(forecast_time, weather_cache.forecast_length)

    for city in forecast_frame.columns:
        fig, ax = matplotlib.pyplot.subplots(len(models), 1, sharex=True)
        real_handle = None
        forecast_handle = None
        model_handles = []
        for model_index, model in enumerate(models):
            local_ax = ax[model_index]

            model_handle = None
            for sample_index, sample_frame in enumerate(past_error_model.samples(num_samples)):
                city_series = sample_frame[sample_frame['City'] == city]
                model_handle = local_ax.plot(city_series['DateTime'], domain_filter(city_series['y'].values), c='blue', alpha=0.3)[0]
            model_handles.append(model_handle)
            real_handle = local_ax.plot(observation_frame[city].index.values, observation_frame[city].values, c='red')[0]
            forecast_handle = local_ax.plot(forecast_frame[city].index.values, forecast_frame[city].values, c='black')[0]

            local_ax.annotate(model_labels[model_index],
                        xy=(0.0, 0.0),
                        xycoords='axes fraction',
                        horizontalalignment='left',
                        verticalalignment='bottom',
                        bbox=BBOX_STYLE.copy())

        ax[int(len(models) / 2)].set_ylabel('Cloud Cover [%]')
        ax[-1].legend((model_handles[0], forecast_handle, real_handle), ('Generated Scenario', 'Forecast Scenario', 'Real Scenario'),
                      ncol=3, loc='upper center', bbox_to_anchor=(0.5, -0.4))
        ax[-1].set_xlabel('Time')

        fig.tight_layout()
        fig.subplots_adjust(bottom=0.15, hspace=0.05)

        matplotlib.pyplot.savefig('{0}_{1}_{2}.png'.format(output_prefix, city, forecast_time.date()))
        matplotlib.pyplot.close(fig)


def example():
    import numpy as np
    import matplotlib.pyplot as pb

    # This functions generate data corresponding to two outputs
    f_output1 = lambda x: 4. * np.cos(x / 5.) - .4 * x - 35. + np.random.rand(x.size)[:, None] * 2.
    f_output2 = lambda x: 6. * np.cos(x / 5.) + .2 * x + 35. + np.random.rand(x.size)[:, None] * 8.

    # {X,Y} training set for each output
    X1 = np.random.rand(100)[:, None];
    X1 = X1 * 75
    X2 = np.random.rand(100)[:, None];
    X2 = X2 * 70 + 30
    Y1 = f_output1(X1)
    Y2 = f_output2(X2)
    # {X,Y} test set for each output
    Xt1 = np.random.rand(100)[:, None] * 100
    Xt2 = np.random.rand(100)[:, None] * 100
    Yt1 = f_output1(Xt1)
    Yt2 = f_output2(Xt2)

    xlim = (0, 100);
    ylim = (0, 50)
    fig = pb.figure(figsize=(12, 8))
    ax1 = fig.add_subplot(211)
    ax1.set_xlim(xlim)
    ax1.set_title('Output 1')
    ax1.plot(X1[:, :1], Y1, 'kx', mew=1.5, label='Train set')
    ax1.plot(Xt1[:, :1], Yt1, 'rx', mew=1.5, label='Test set')
    ax1.legend()
    ax2 = fig.add_subplot(212)
    ax2.set_xlim(xlim)
    ax2.set_title('Output 2')
    ax2.plot(X2[:, :1], Y2, 'kx', mew=1.5, label='Train set')
    ax2.plot(Xt2[:, :1], Yt2, 'rx', mew=1.5, label='Test set')
    ax2.legend()

    matplotlib.pyplot.show()

    def plot_2outputs(m, xlim, ylim):
        fig = pb.figure(figsize=(12, 8))
        # Output 1
        ax1 = fig.add_subplot(211)
        ax1.set_xlim(xlim)
        ax1.set_title('Output 1')
        m.plot(plot_limits=xlim, fixed_inputs=[(1, 0)], which_data_rows=slice(0, 100), ax=ax1)
        ax1.plot(Xt1[:, :1], Yt1, 'rx', mew=1.5)
        # Output 2
        ax2 = fig.add_subplot(212)
        ax2.set_xlim(xlim)
        ax2.set_title('Output 2')
        m.plot(plot_limits=xlim, fixed_inputs=[(1, 1)], which_data_rows=slice(100, 200), ax=ax2)
        ax2.plot(Xt2[:, :1], Yt2, 'rx', mew=1.5)

    import GPy
    K = GPy.kern.RBF(1)
    B = GPy.kern.Coregionalize(input_dim=1, output_dim=2)
    multkernel = K.prod(B, name='B.K')
    print(multkernel)

    # Components of B
    print('W matrix\n', B.W)
    print('\nkappa vector\n', B.kappa)
    print('\nB matrix\n', B.B)

    K = GPy.kern.Matern32(1)
    icm = GPy.util.multioutput.ICM(input_dim=1, num_outputs=2, kernel=K)

    m = GPy.models.GPCoregionalizedRegression([X1, X2], [Y1, Y2], kernel=icm)
    m['.*Mat32.var'].constrain_fixed(1.)  # For this kernel, B.kappa encodes the variance now.
    m.optimize()
    print(m)
    plot_2outputs(m, xlim=(0, 100), ylim=(-20, 60))

    matplotlib.pyplot.show()

    print('here')

    icm = GPy.util.multioutput.ICM(input_dim=1, num_outputs=2, kernel=GPy.kern.RBF(1))
    print(icm)

    K = GPy.kern.Matern32(1)

    m1 = GPy.models.GPRegression(X1, Y1, kernel=K.copy())
    m1.optimize()
    m2 = GPy.models.GPRegression(X2, Y2, kernel=K.copy())
    m2.optimize()
    fig = pb.figure(figsize=(12, 8))
    # Output 1
    ax1 = fig.add_subplot(211)
    m1.plot(plot_limits=xlim, ax=ax1)
    ax1.plot(Xt1[:, :1], Yt1, 'rx', mew=1.5)
    ax1.set_title('Output 1')
    # Output 2
    ax2 = fig.add_subplot(212)
    m2.plot(plot_limits=xlim, ax=ax2)
    ax2.plot(Xt2[:, :1], Yt2, 'rx', mew=1.5)
    ax2.set_title('Output 2')


def train_gp_command(args):
    # example()

    import numpy.random
    import matplotlib.pyplot as pb

    # #build a design matrix with a column of integers indicating the output
    # X1 = np.random.rand(50, 1) * 8
    # X2 = np.random.rand(30, 1) * 5
    #
    # #build a suitable set of observed variables
    # Y1 = np.sin(X1) + np.random.randn(*X1.shape) * 0.05
    # Y2 = np.sin(X2) + np.random.randn(*X2.shape) * 0.05 + 2.
    #
    # m = GPy.models.SparseGPCoregionalizedRegression(X_list=[X1,X2], Y_list=[Y1,Y2])
    # m.optimize('bfgs', max_iters=100)
    #
    # slices = GPy.util.multioutput.get_slices([X1,X2])
    # m.plot(fixed_inputs=[(1,0)],which_data_rows=slices[0],Y_metadata={'output_index':0})
    # m.plot(fixed_inputs=[(1,1)],which_data_rows=slices[1],Y_metadata={'output_index':1},ax=pb.gca())
    # pb.ylim(-3,)
    # matplotlib.pyplot.show()

    weather_cache = WeatherCache()
    weather_cache.load()

    forecast = weather_cache.get_forecast_frame(datetime.datetime(2019, 7, 1), datetime.timedelta(days=5))
    cities = [quake.city.LONDON, quake.city.IPSWICH]  # , quake.city.BIRMINGHAM, quake.city.BRISTOL, quake.city.CAMBRIDGE]

    X = numpy.expand_dims(numpy.arange(len(forecast)), axis=1)
    Y1 = numpy.expand_dims(forecast[quake.city.LONDON].values.astype(float), axis=1)
    Y2 = numpy.expand_dims(forecast[quake.city.IPSWICH].values.astype(float), axis=1)

    import matplotlib.pyplot as pb

    fig = pb.figure(figsize=(12, 8))
    ax1 = fig.add_subplot(211)
    ax1.set_title('Output 1')
    ax1.plot(X, Y1, 'kx', mew=1.5, label='Train set')
    ax1.legend()
    ax2 = fig.add_subplot(212)
    ax2.set_title('Output 2')
    ax2.plot(X, Y2, 'kx', mew=1.5, label='Train set')
    ax2.legend()

    matplotlib.pyplot.show(block=True)

    def plot_2outputs(m):
        fig = pb.figure(figsize=(12, 8))
        # Output 1
        ax1 = fig.add_subplot(211)
        ax1.set_title('Output 1')
        m.plot(fixed_inputs=[(1, 0)], which_data_rows=slice(0, 100), ax=ax1)
        ax1.plot(X, Y1, 'rx', mew=1.5)
        # Output 2
        ax2 = fig.add_subplot(212)
        ax2.set_title('Output 2')
        m.plot(fixed_inputs=[(1, 1)], which_data_rows=slice(100, 200), ax=ax2)
        ax2.plot(X, Y2, 'rx', mew=1.5)

    #
    import GPy
    K = GPy.kern.RBF(1)
    B = GPy.kern.Coregionalize(input_dim=1, output_dim=2)
    multkernel = K.prod(B, name='B.K')
    print(multkernel)

    # Components of B
    print('W matrix\n', B.W)
    print('\nkappa vector\n', B.kappa)
    print('\nB matrix\n', B.B)

    K = GPy.kern.Matern32(1)
    icm = GPy.util.multioutput.ICM(input_dim=1, num_outputs=2, kernel=K)

    m = GPy.models.GPCoregionalizedRegression([X, X], [Y1, Y2], kernel=icm)
    m['.*Mat32.var'].constrain_fixed(1.)  # For this kernel, B.kappa encodes the variance now.
    m.optimize()
    print(m)
    plot_2outputs(m)
    #
    matplotlib.pyplot.show(block=True)

    print('test')


def save_figure(fig, file_name):
    fig.savefig('{0}.png'.format(file_name))


def plot_command(args):
    from_arg = getattr(args, 'from')
    if from_arg:
        left_time_limit = datetime.datetime.strptime(from_arg, '%Y-%m-%d')
    else:
        left_time_limit = None

    weather_cache = WeatherCache()
    weather_cache.load()

    forecast_frame = weather_cache.forecast_frame
    if left_time_limit:
        forecast_frame = forecast_frame[forecast_frame['DateTime'] > left_time_limit].copy()

    cities = forecast_frame['City'].unique()
    for forecast_distance in [pandas.Timedelta(days=1), pandas.Timedelta(days=2),
                              pandas.Timedelta(days=3), pandas.Timedelta(days=4)]:
        for city in cities:
            forecast_hours = forecast_distance.total_seconds() / matplotlib.dates.SEC_PER_HOUR

            data_frame = forecast_frame[forecast_frame['City'] == city].copy()

            figure, axis = matplotlib.pyplot.subplots(1, 1)

            observed_series = data_frame[data_frame['Delay'] == pandas.Timedelta(seconds=0)].copy()
            observed_series.set_index('DateTime', inplace=True)
            observed_series.sort_index(inplace=True)

            forecast_series = data_frame[data_frame['Delay'] == forecast_distance].copy()
            forecast_series.set_index('DateTime', inplace=True)
            forecast_series.sort_index(inplace=True)

            axis.plot(observed_series['CloudCover'], '.', label='Observed')
            axis.plot(forecast_series['CloudCover'], '--', label='Forecast {0}'.format(int(forecast_hours)))
            axis.xaxis.set_tick_params(rotation=90)
            axis.set_title(city.name)
            axis.legend(loc='lower right')
            figure.tight_layout()
            matplotlib.pyplot.savefig('cloud_cover_forecast_{0}_{1}.png'.format(city.name, int(forecast_hours)))
            matplotlib.pyplot.close(figure)


def simple_date_str(date_time):
    return date_time.strftime('%Y-%m-%d')


def plot_forecast_command(args):
    from_date_time = getattr(args, 'from')
    to_date_time = getattr(args, 'to')
    output_prefix = getattr(args, 'output_prefix')

    weather_cache = WeatherCache()
    weather_cache.load()

    duration = to_date_time - from_date_time

    def get_diff_frame(time_point):
        forecast_frame = weather_cache.get_forecast_frame(time_point, duration)
        weather_frame = weather_cache.get_observation_frame(time_point, duration)
        diff_frame = forecast_frame - weather_frame
        diff_frame.dropna(inplace=True)
        diff_frame['Delay'] = diff_frame.index - time_point
        return diff_frame

    time_points = weather_cache.forecast_frame['DateTime'].unique()
    time_points = list({pandas.to_datetime(date_time) for date_time in time_points})
    time_points.sort()

    diff_frames = [get_diff_frame(time_point) for time_point in time_points]
    diff_frame = pandas.concat(diff_frames, ignore_index=True)
    delay_values = list(pandas.to_timedelta(value, unit='ns') for value in diff_frame['Delay'].unique())
    delay_values.sort()

    error_data = []
    for delay_value in delay_values:
        delay_diff_frame = diff_frame[diff_frame['Delay'] == delay_value]

        for city_column in delay_diff_frame.columns:
            if not isinstance(city_column, quake.city.City):
                continue
            for error_value in delay_diff_frame[city_column]:
                error_data.append([city_column, pandas.Timedelta(value=delay_value.total_seconds(), unit='s'), error_value])
    columns = ['City', 'Delay', 'Error']
    error_frame = pandas.DataFrame(columns=columns, data=error_data)

    delay_values = error_frame['Delay'].unique()
    rows = []
    for delay_value in delay_values:
        row = dict()
        filter_frame = error_frame[error_frame['Delay'] == delay_value].copy()
        for city in filter_frame['City'].unique():
            error_series = filter_frame[filter_frame['City'] == city]['Error']
            aggregate = numpy.std(error_series)
            row[city] = aggregate
        rows.append(row)
    std_error_frame = pandas.DataFrame(index=delay_values, data=rows)

    def date_axis_formatter(x, pos):
        date_time = pandas.Timestamp.fromordinal(x.astype(numpy.int64))
        return date_time.date()

    forecast_frame = weather_cache.get_forecast_frame(from_date_time, duration)
    observation_frame = weather_cache.get_observation_frame(from_date_time, duration)
    for city in forecast_frame.columns:
        if not isinstance(city, quake.city.City):
            continue

        figure, ax = matplotlib.pyplot.subplots()

        x = forecast_frame[city].index
        y1 = forecast_frame[city] + 1.96 * std_error_frame[city].values
        y2 = forecast_frame[city] - 1.96 * std_error_frame[city].values
        ax.fill_between(x, y1.values, y2.values, color=matplotlib.colors.CSS4_COLORS['lightgrey'], alpha=0.5)
        ax.plot(x, y1, ls='--', color=matplotlib.colors.CSS4_COLORS['grey'])
        ax.plot(x, y2, ls='--', color=matplotlib.colors.CSS4_COLORS['grey'], label='95% Confidence Interval')
        ax.plot(forecast_frame[city], label='Forecast 5-days')
        ax.plot(observation_frame[city], label='Observation')
        ax.legend(loc='lower right')
        ax.set_title(city.name)
        ax.set_xlabel('Time')
        ax.set_ylim(-10, 110)
        ax.xaxis.set_tick_params(rotation=90)
        ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(date_axis_formatter))
        ax.set_ylabel('Cloud Cover [%]')
        figure.tight_layout()

        save_figure('{0}_{1}_{2}_{3}'.format(output_prefix, city.name, simple_date_str(from_date_time), simple_date_str(to_date_time)))


def plot_coregionalization(args):
    forecast_start_time = getattr(args, 'forecast_start_time')
    observation_start_time = getattr(args, 'observation_start_time')
    output_prefix = getattr(args, 'output_prefix')

    weather_cache = WeatherCache()
    weather_cache.load()

    forecast_duration = datetime.timedelta(days=5)
    observation_duration = forecast_start_time - observation_start_time
    forecast_frame = weather_cache.get_forecast_frame(forecast_start_time, forecast_duration)
    observation_frame = weather_cache.get_observation_frame(observation_start_time, observation_duration)
    cities = forecast_frame.columns

    MEAN_COLOR = matplotlib.colors.CSS4_COLORS['yellowgreen']
    FORECAST_COLOR = matplotlib.colors.CSS4_COLORS['blue']
    OBSERVATION_COLOR = matplotlib.colors.CSS4_COLORS['green']
    CONFIDENCE_COLOR = matplotlib.colors.CSS4_COLORS['silver']

    # plot input data
    fig, ax = matplotlib.pyplot.subplots(len(cities), 1, sharex=True, figsize=(10, 8))
    observation_handle, forecast_handle = None, None
    for city_index, city in enumerate(cities):
        observation_handle = ax[city_index].scatter(observation_frame[city].index, observation_frame[city], marker='s', s=1, color=OBSERVATION_COLOR)
        forecast_handle = ax[city_index].scatter(forecast_frame[city].index, forecast_frame[city], marker='s', s=1, color=FORECAST_COLOR)
        ax[city_index].set_ylim(-25, 125)

        at = matplotlib.offsetbox.AnchoredText(city.name, frameon=False, loc='lower left')
        ax[city_index].add_artist(at)

    for tick in ax[-1].get_xticklabels():
        tick.set_rotation(90)
    ax[int(len(cities) / 2)].set_ylabel('Cloud Cover [%]')
    ax[-1].set_xlabel('Date')

    legend_artist = ax[-1].legend([observation_handle, forecast_handle], ['Observation', 'Forecast'],
                                  ncol=2, loc='center', bbox_to_anchor=(0.5, -2))
    ax[-1].add_artist(legend_artist)

    fig.tight_layout()
    fig.subplots_adjust(bottom=0.2, hspace=0.10)

    save_figure(fig, '{0}_{1}_input'.format(output_prefix, simple_date_str(forecast_start_time)))
    matplotlib.pyplot.close(fig)

    model = CoregionalizationModel(observation_frame, forecast_frame)
    fitted_frame = model.optimize()
    sample_frames = model.samples(5)

    mean_handle, confidence_handle, observation_handle, forecast_handle = None, None, None, None
    sample_handles = []
    fig, ax = matplotlib.pyplot.subplots(len(cities), 1, sharex=True, figsize=(10, 8))
    for city_index, city in enumerate(cities):
        city_frame = fitted_frame[fitted_frame['City'] == city]
        confidence_handle = ax[city_index].fill_between(city_frame['DateTime'],
                                                        city_frame['yhat_upper'],
                                                        city_frame['yhat_lower'],
                                                        color=CONFIDENCE_COLOR, alpha=0.5)
        mean_handle = ax[city_index].plot(city_frame['DateTime'], city_frame['yhat'], color=MEAN_COLOR)[0]

        sample_handles = []
        for sample_frame in sample_frames:
            city_sample_frame = sample_frame[sample_frame['City'] == city]
            sample_handle = ax[city_index].plot(city_sample_frame['DateTime'], city_sample_frame['y'])[0]
            sample_handles.append(sample_handle)

        at = matplotlib.offsetbox.AnchoredText(city.name, frameon=False, loc='lower left')
        ax[city_index].add_artist(at)
        ax[city_index].set_ylim(-25, 125)

        observation_city_series = observation_frame[city]
        forecast_city_series = forecast_frame[city]
        observation_handle = ax[city_index].scatter(observation_city_series.index, observation_city_series.values,
                                                    marker='s', s=1.0, color=OBSERVATION_COLOR)
        forecast_handle = ax[city_index].scatter(forecast_city_series.index, forecast_city_series.values,
                                                 marker='s', s=1.0, color=FORECAST_COLOR)
    for tick in ax[-1].get_xticklabels():
        tick.set_rotation(90)
    ax[int(len(cities) / 2)].set_ylabel('Cloud Cover [%]')
    ax[-1].set_xlabel('Date')
    legend_artist = ax[-1].legend([mean_handle, confidence_handle, observation_handle, forecast_handle,
                                   sample_handles[0], sample_handles[1], sample_handles[2], sample_handles[3]],
                                  ['Mean', 'Confidence', 'Observation', 'Forecast', 'Sample 1', 'Sample 2', 'Sample 3', 'Sample 4'],
                                  ncol=4, loc='center', bbox_to_anchor=(0.5, -2.1))
    ax[-1].add_artist(legend_artist)
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.2, hspace=0.10)

    save_figure(fig, '{0}_{1}_output'.format(output_prefix, simple_date_str(forecast_start_time)))
    matplotlib.pyplot.close(fig)


if __name__ == '__main__':
    args = parse_args()
    command = getattr(args, 'command')

    if command == BUILD_CACHE_COMMAND:
        build_cache_command(args)
    elif command == PLOT_COMMAND:
        plot_command(args)
    elif command == COVARIANCE_COMMAND:
        covariance_command(args)
    elif command == VAR_COMMAND:
        compute_vector_auto_regression(args)
    elif command == EXTEND_COMMAND:
        extend_problem_definition(args)
    elif command == GENERATE_COMMAND:
        generate_command(args)
    elif command == TRAIN_GP_COMMAND:
        train_gp_command(args)
    elif command == PLOT_FORECAST_COMMAND:
        plot_forecast_command(args)
    elif command == PLOT_COREGIONALIZATION_COMMAND:
        plot_coregionalization(args)
    elif command == PLOT_GENERATED_SCENARIOS_COMMAND:
        plot_generated_scenarios(args)


    def plot_errors(station, begin_start_time, end_start_time):
        weather_cache = WeatherCache()
        weather_cache.load()

        period_start = begin_start_time
        period_length = datetime.timedelta(days=5)
        while period_start <= end_start_time:
            forecast_frame = weather_cache.get_forecast_frame(period_start, period_length)
            observation_frame = weather_cache.get_observation_frame(period_start, period_length)
            error_frame = forecast_frame - observation_frame
            error_frame.dropna(inplace=True)
            error_frame.reset_index(inplace=True)
            if not error_frame.empty:
                error_frame[station].plot(style='-')
            period_start += datetime.timedelta(days=1)
        matplotlib.pyplot.show(block=True)


    def plot_pacf(station, start_time, duration):
        weather_cache = WeatherCache()
        weather_cache.load()

        forecast_frame = weather_cache.get_forecast_frame(start_time, duration)
        observation_frame = weather_cache.get_observation_frame(start_time, duration)
        error_frame = forecast_frame - observation_frame
        error_frame.dropna(inplace=True)
        error_frame.reset_index(inplace=True)

        import statsmodels.graphics.tsaplots
        statsmodels.graphics.tsaplots.plot_pacf(error_frame[quake.city.LONDON].values.transpose())
        matplotlib.pyplot.show(block=True)

        # def test_multiout_regression_md():
        #     import GPy
        #     import numpy as np
        #
        #     np.random.seed(0)
        #
        #     N = 20
        #     N_train = 5
        #     D = 8
        #     noise_var = 0.3
        #
        #     k = GPy.kern.RBF(1, lengthscale=0.1)
        #     x_raw = np.random.rand(N * D, 1)
        #
        #     # dimension assignment
        #     D_list = []
        #     for i in range(2):
        #         while True:
        #             D_sub_list = []
        #             ratios = []
        #             r_p = 0.
        #             for j in range(3):
        #                 ratios.append(np.random.rand() * (1 - r_p) + r_p)
        #                 D_sub_list.append(int((ratios[-1] - r_p) * 4 * N_train))
        #                 r_p = ratios[-1]
        #             D_sub_list.append(4 * N_train - np.sum(D_sub_list))
        #             if (np.array(D_sub_list) != 0).all():
        #                 D_list.extend([a + N - N_train for a in D_sub_list])
        #                 break
        #
        #     cov = k.K(x_raw)
        #
        #     k_r = GPy.kern.RBF(2, lengthscale=.4)
        #     x_r = np.random.rand(D, 2)
        #     cov_r = k_r.K(x_r)
        #
        #     cov_all = np.repeat(np.repeat(cov_r, D_list, axis=0), D_list, axis=1) * cov
        #     L = GPy.util.linalg.jitchol(cov_all)
        #
        #     y_latent = L.dot(np.random.randn(N * D))
        #
        #     x = np.zeros((D * N_train,))
        #     y = np.zeros((D * N_train,))
        #     x_test = np.zeros((D * (N - N_train),))
        #     y_test = np.zeros((D * (N - N_train),))
        #     indexD = np.zeros((D * N_train), dtype=np.int)
        #     indexD_test = np.zeros((D * (N - N_train)), dtype=np.int)
        #
        #     offset_all = 0
        #     offset_train = 0
        #     offset_test = 0
        #     for i in range(D):
        #         D_test = N - N_train
        #         D_train = D_list[i] - N + N_train
        #         y[offset_train:offset_train + D_train] = y_latent[offset_all:offset_all + D_train]
        #         x[offset_train:offset_train + D_train] = x_raw[offset_all:offset_all + D_train, 0]
        #         y_test[offset_test:offset_test + D_test] = y_latent[offset_all + D_train:offset_all + D_train + D_test]
        #         x_test[offset_test:offset_test + D_test] = x_raw[offset_all + D_train:offset_all + D_train + D_test, 0]
        #         indexD[offset_train:offset_train + D_train] = i
        #         indexD_test[offset_test:offset_test + D_test] = i
        #         offset_train += D_train
        #         offset_test += D_test
        #         offset_all += D_train + D_test
        #
        #     y_noisefree = y.copy()
        #     y += np.random.randn(*y.shape) * np.sqrt(noise_var)
        #     x_flat = x.flatten()[:, None]
        #     y_flat = y.flatten()[:, None]
        #
        #     Mr, Mc, Qr, Qc = 4, 3, 2, 1
        #
        #     m = GPy.models.GPMultioutRegressionMD(x_flat, y_flat, indexD, Xr_dim=Qr, kernel_row=GPy.kern.RBF(Qr, ARD=False), num_inducing=(Mc, Mr))
        #     m.optimize_auto(max_iters=1)
        #     m.randomize()
        #     assert m.checkgrad()
        #
        #     m = GPy.models.GPMultioutRegressionMD(x_flat, y_flat, indexD, Xr_dim=Qr, kernel_row=GPy.kern.RBF(Qr, ARD=False), num_inducing=(Mc, Mr),
        #                                           init='rand')
        #     m.optimize_auto(max_iters=1)
        #     m.randomize()
        #     assert m.checkgrad()
        #
        #
        # def test_multiout_regression():
        #     import numpy as np
        #
        #     np.random.seed(0)
        #     import GPy
        #
        #     N = 10
        #     N_train = 5
        #     D = 4
        #     noise_var = .3
        #
        #     k = GPy.kern.RBF(1, lengthscale=0.1)
        #     x = np.random.rand(N, 2)
        #     cov = k.K(x)
        #
        #     k_r = GPy.kern.RBF(2, lengthscale=.4)
        #     x_r = np.random.rand(D, 2)
        #     cov_r = k_r.K(x_r)
        #
        #     cov_all = np.kron(cov_r, cov)
        #     L = GPy.util.linalg.jitchol(cov_all)
        #
        #     y_latent = L.dot(np.random.randn(N * D)).reshape(D, N).T
        #
        #     x_test = x[N_train:]
        #     y_test = y_latent[N_train:]
        #     x = x[:N_train]
        #     y = y_latent[:N_train] + np.random.randn(N_train, D) * np.sqrt(noise_var)
        #
        #     Mr = D
        #     Mc = x.shape[0]
        #     Qr = 5
        #     Qc = x.shape[1]
        #
        #     m_mr = GPy.models.GPMultioutRegression(x, y, Xr_dim=Qr, kernel_row=GPy.kern.RBF(Qr, ARD=True), num_inducing=(Mc, Mr), init='GP')
        #     m_mr.optimize_auto(max_iters=1)
        #     m_mr.randomize()
        #     assert m_mr.checkgrad()
        #
        #     m_mr = GPy.models.GPMultioutRegression(x, y, Xr_dim=Qr, kernel_row=GPy.kern.RBF(Qr, ARD=True), num_inducing=(Mc, Mr), init='rand')
        #     m_mr.optimize_auto(max_iters=1)
        #     m_mr.randomize()
        #     assert m_mr.checkgrad()
        #
        #     mean, variance = m_mr.predict(x)
        #     quantiles = m_mr.predict_quantiles(x)
        #
        #     print('here')
        #
        #
        # def my_test_regression_with_missing_values():
        #     weather_cache = WeatherCache()
        #     weather_cache.load()
        #
        #     forecasts = [weather_cache.get_observation_frame(datetime.datetime(2019, 6, 20), weather_cache.forecast_length),
        #                  weather_cache.get_observation_frame(datetime.datetime(2019, 6, 25), weather_cache.forecast_length),
        #                  weather_cache.get_observation_frame(datetime.datetime(2019, 7, 1), weather_cache.forecast_length),
        #                  weather_cache.get_observation_frame(datetime.datetime(2019, 7, 5), weather_cache.forecast_length),
        #                  weather_cache.get_observation_frame(datetime.datetime(2019, 7, 10), weather_cache.forecast_length),
        #                  weather_cache.get_observation_frame(datetime.datetime(2019, 7, 15), weather_cache.forecast_length)]
        #
        #     restricted_cities = forecasts[0].columns
        #
        #     X = []
        #     Y = []
        #     D_index = []
        #     for forecast_index, forecast in enumerate(forecasts):
        #         min_start_time = forecast.index.min()
        #         for city_index, city in enumerate(restricted_cities):
        #             for time_index, cloud_cover in forecast[city].items():
        #                 X.append([city_index, int((time_index - min_start_time).total_seconds() // 3600)])
        #                 Y.append([cloud_cover])
        #                 D_index.append(forecast_index)
        #     X = numpy.array(X)
        #     Y = numpy.array(Y)
        #     D_index = numpy.array(D_index)
        #
        #     Mr = len(forecasts)
        #     Mc = len(forecasts[0]) * len(restricted_cities)
        #     Qr = len(forecasts)
        #     Qc = 2
        #
        #     m = GPy.models.GPMultioutRegressionMD(X, Y, D_index, Xr_dim=Qr, kernel_row=GPy.kern.RBF(Qr, ARD=False), num_inducing=(Mc, Mr), init='GP')
        #     m.optimize_auto(max_iters=1)
        #     m.randomize()
        #     assert m.checkgrad()
        #
        #


    # def my_test_error_regression_md():
    #     weather_cache = WeatherCache()
    #     weather_cache.load()
    #
    #     import pickle
    #     if os.path.exists('error_frames.pickle'):
    #         with open('error_frames.pickle', 'rb') as input_stream:
    #             error_frames = pickle.load(input_stream)
    #     else:
    #         error_frames = weather_cache.get_error_frames()
    #         with open('error_frames.pickle', 'wb') as output_stream:
    #             pickle.dump(error_frames, output_stream)
    #
    #     assert error_frames
    #     # error_frames = error_frames[:5]
    #
    #     data = []
    #     for frame_index, error_frame in enumerate(error_frames):
    #         error_frame_start = error_frame.index.min()
    #         for city_index, city in enumerate(error_frame.columns):
    #             city_series = error_frame[city]
    #             for time_index, error in city_series.items():
    #                 data.append(
    #                     [frame_index, city, city_index, time_index, int((time_index - error_frame_start).total_seconds() // 3600), error])
    #     data_frame = pandas.DataFrame(data=data, columns=['SampleIndex', 'City', 'CityIndex', 'DateTime', 'Delay', 'CloudCover'])
    #     del data
    #
    #     city_indices = list(data_frame['CityIndex'].unique())
    #     city_indices.sort()
    #
    #     delays = list(data_frame['Delay'].unique())
    #     delays.sort()
    #
    #     X = []
    #     Y = []
    #
    #     for city_index in city_indices:
    #         for delay in delays:
    #             city_delay_frame = data_frame[(data_frame['CityIndex'] == city_index) & (data_frame['Delay'] == delay)]
    #             cloud_cover_frame = city_delay_frame[['SampleIndex', 'CloudCover']]
    #             cloud_cover_frame.set_index('SampleIndex', inplace=True)
    #             cloud_cover_frame.sort_index(inplace=True)
    #             y = cloud_cover_frame.values.flatten().tolist()
    #             X.append([city_index, delay])
    #             Y.append(y)
    #     X = numpy.array(X)
    #     Y = numpy.array(Y)
    #
    #     Mr = Y.shape[1]
    #     Mc = 10  # int(X.shape[0] * 0.05)
    #     Qr = 10  # int(X.shape[0] * 0.05)
    #
    #     model = GPy.models.GPMultioutRegression(X, Y, Xr_dim=Qr, kernel_row=GPy.kern.RBF(Qr, ARD=False), num_inducing=(Mc, Mr), init='GP')
    #     model.optimize_auto(max_iters=1000)
    #
    #     model.plot_f()
    #     matplotlib.pyplot.show(block=True)

    # mean, variance = model.predict(X)
    # quantiles = model.predict_quantiles(X)

    # m_mr.randomize()
    # assert m_mr.checkgrad()

    # sample from posterior - print examples
    # samples = model.posterior_samples_f(X, size=10)

    #
    #
    # test_multiout_regression()

    # my_test_error_regression_md()

    # test_multiout_regression_md()
    # test_multiout_regression_md()

    # test_multiout_regression()

    # my_test_error_regression_md()

    def heteroscedastatic_example():
        import GPy.likelihoods
        import GPy.likelihoods.link_functions

        import matplotlib.pyplot as plt
        import numpy as np

        N = 40
        X = np.random.uniform(low=0, high=10, size=(N, 1)).flatten().tolist()

        X.sort()
        X = np.array(X).reshape(-1, 1)

        V = np.random.uniform(0.1, 3.0, size=(N, 1)) ** 2
        Y = np.sin(X) + np.random.normal(size=(N, 1)) * np.sqrt(V)
        x1 = np.linspace(0, 10, 1000)

        m = GPy.models.GPHeteroscedasticRegression(X, Y, GPy.kern.RBF(1))
        m['.*het_Gauss.variance'] = V  # Set the noise parameters to the error in Y
        m.het_Gauss.variance.fix()  # We can fix the noise term, since we already know it
        m.optimize()

        fig, ax = matplotlib.pyplot.subplots(1, 1)
        m.plot(plot_density=False, plot_data=False, ax=ax)

        ax.plot(x1, np.sin(x1), 'r')

        num_samples = 10
        samples = m.posterior_samples_f(X, size=num_samples)
        for sample_index in range(num_samples):
            prediction = samples[:, :, sample_index] + np.random.normal(size=(N, 1)) * np.sqrt(V)
            ax.plot(X.flatten(), prediction.flatten())
        matplotlib.pyplot.show(block=True)


    def modelling_errors_independenty():
        weather_cache = WeatherCache()
        weather_cache.load()

        import pickle

        if os.path.exists('error_frames.pickle'):
            with open('error_frames.pickle', 'rb') as input_stream:
                error_frames = pickle.load(input_stream)
        else:
            error_frames = weather_cache.get_error_frames()
            with open('error_frames.pickle', 'wb') as output_stream:
                pickle.dump(error_frames, output_stream)

        X = []
        Y = []

        # fig, ax = matplotlib.pyplot.subplots(1, 1)
        # for error_frame in error_frames:
        #     city_frame = error_frame[quake.city.LONDON].to_frame()
        #     city_frame['Delay'] = city_frame.index - city_frame.index.min()
        #     city_frame['Delay'] = city_frame['Delay'].apply(lambda value: int(value.total_seconds() / 3600))
        #     ax.scatter(city_frame['Delay'], city_frame[quake.city.LONDON])
        # matplotlib.pyplot.show(block=True)

        partial_frames = []
        for error_frame in error_frames:
            working_frame = error_frame.copy()
            min_date_time = working_frame.index.min()
            working_frame['DateTime'] = working_frame.index
            working_frame['Delay'] = working_frame['DateTime'].apply(lambda value: int((value - min_date_time).total_seconds() / 3600))
            partial_frames.append(working_frame)

        error_frame = pandas.concat(partial_frames, ignore_index=True)
        error_frame.sort_values(by=['Delay', 'DateTime'], inplace=True)

        Y = []
        X = []

        delays = list(error_frame['Delay'].unique())
        delays.sort()

        Y_mean = []
        Y_std = []

        for delay in delays:
            series = error_frame[error_frame['Delay'] == delay][quake.city.LONDON]
            mean = series.mean()
            std = series.std()
            for value in series:
                # if std != 0.0:
                #     Y.append((value - mean) / std)
                # else:
                #     Y.append((value - mean))
                Y.append(value)
                X.append(delay)
            Y_mean.append([mean])
            Y_std.append([std if std > 0.0 else 1.0])

        Y_mean = numpy.array(Y_mean)
        Y_std = numpy.array(Y_std)

        Y = numpy.array(Y).reshape(-1, 1)
        X = numpy.array(X).reshape(-1, 1)
        kern = GPy.kern.Matern32(1) + GPy.kern.White(1, variance=81) + GPy.kern.Bias(1)  # + GPy.kern.Linear(1, 1, active_dims=[0])

        model = GPy.models.GPRegression(X, Y, kern, normalizer=True)
        model.optimize(messages=True)

        num_samples = 30
        X_sample = numpy.array(delays).reshape(-1, 1)
        samples = model.posterior_samples_f(X_sample, size=num_samples, full_cov=True)

        model.plot(samples=1)
        matplotlib.pyplot.show(block=True)

        fig, ax = matplotlib.pyplot.subplots(1, 1)
        for error_frame in error_frames:
            city_frame = error_frame[quake.city.LONDON].to_frame()
            city_frame['Delay'] = city_frame.index - city_frame.index.min()
            city_frame['Delay'] = city_frame['Delay'].apply(lambda value: int(value.total_seconds() / 3600))
            ax.plot(city_frame['Delay'], city_frame[quake.city.LONDON], alpha=0.5)

        for sample_index in range(num_samples):
            inverse_correction = samples[:, :, sample_index]  # numpy.multiply(samples[:, :, sample_index], Y_std) + Y_mean
            ax.plot(X_sample.flatten(), inverse_correction.flatten(), c='black')

        matplotlib.pyplot.show(block=True)
        print('here')
