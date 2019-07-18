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
import copy
import datetime
import json
import logging
import os
import subprocess
import warnings
import sys

import matplotlib.dates
import matplotlib.pyplot
import matplotlib.ticker
import matplotlib.colors
import numpy
import pandas
import quake.city
import tabulate
import tqdm

# TODO: build confidence intervals on error - are errors correlated between series?
# TODO: plot confidence interval for weather at given day
# TODO: read how others forecast weather
# TODO: study gaussian process, multi-output gaussian process, non-parametric gaussian regression

BUILD_CACHE_COMMAND = 'build-cache'
PLOT_COMMAND = 'plot-cloud-cover'
PLOT_FORECAST_COMMAND = 'plot-forecast'
COVARIANCE_COMMAND = 'compute-covariance'
VAR_COMMAND = 'compute-var'
EXTEND_COMMAND = 'extend'
GENERATE_COMMAND = 'generate'
DATE_FORMAT = '%Y-%m-%d'


class WeatherCache:
    FILE_TABLE = 'a'
    FILE_MODE = 'w'
    FORECAST_CACHE_FILE = 'forecast.hdf'
    OBSERVATION_CACHE_FILE = 'observation.hdf'
    ZERO_TIME_DELTA = datetime.timedelta()

    def __init__(self):
        self.__forecast_frame = None
        self.__observation_frame = None

    def rebuild(self):

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

        observation_frame = load_observation_frame('/home/pmateusz/dev/quake/data/weather/ground_station_data_set_filled.csv')

        self.__forecast_frame = forecast_frame
        self.__observation_frame = observation_frame

    def load(self):
        self.__forecast_frame = pandas.read_hdf(self.FORECAST_CACHE_FILE, self.FILE_TABLE)
        self.__observation_frame = pandas.read_hdf(self.OBSERVATION_CACHE_FILE, self.FILE_TABLE)

    def save(self):
        self.__forecast_frame.to_hdf(self.FORECAST_CACHE_FILE, self.FILE_TABLE, mode=self.FILE_MODE)
        self.__observation_frame.to_hdf(self.OBSERVATION_CACHE_FILE, self.FILE_TABLE, mode=self.FILE_MODE)

    def get_forecast_frame(self, start_time, duration):
        end_time = start_time + duration

        filter_frame \
            = self.__forecast_frame[(self.__forecast_frame['DateTime'] >= start_time) & (self.__forecast_frame['DateTime'] <= end_time)].copy()
        filter_frame['ForecastDateTime'] = filter_frame['DateTime'] - filter_frame['Delay']
        filter_frame['ForecastDateTimeDiff'] = filter_frame['ForecastDateTime'].apply(lambda value: abs((value - start_time).total_seconds()))
        min_forecast_date_time_diff = filter_frame['ForecastDateTimeDiff'].min()
        forecast_date_time = filter_frame[filter_frame['ForecastDateTimeDiff'] == min_forecast_date_time_diff]['ForecastDateTime'].iloc[0]

        data_frame = filter_frame[filter_frame['ForecastDateTime'] == forecast_date_time].copy()
        self.__verify_period(data_frame, start_time, end_time)
        return self.__pivot_transform(data_frame)

    def get_observation_frame(self, start_time, duration):
        end_time = start_time + duration

        data_frame = self.__forecast_frame[(self.__forecast_frame['Delay'] == self.ZERO_TIME_DELTA)
                                           & (self.__forecast_frame['DateTime'] >= start_time)
                                           & (self.__forecast_frame['DateTime'] <= end_time)].copy()
        self.__verify_period(data_frame, start_time, end_time)
        return self.__pivot_transform(data_frame)

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

    generate_parser = sub_parsers.add_parser(GENERATE_COMMAND)
    generate_parser.add_argument('--from')
    generate_parser.add_argument('--to')
    generate_parser.add_argument('--problem-prefix')

    plot_forecast_parser = sub_parsers.add_parser(PLOT_FORECAST_COMMAND)
    plot_forecast_parser.add_argument('--from', action=ParseDateAction)
    plot_forecast_parser.add_argument('--to', action=ParseDateAction)

    compute_var = sub_parsers.add_parser(VAR_COMMAND)

    return parser.parse_args()


def build_cache_command(args):
    weather_cache = WeatherCache()
    weather_cache.rebuild()
    weather_cache.save()


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

    with open(problem_file, 'r') as input_stream:
        json_object = json.load(input_stream)
        problem = Problem(json_object)

    weather_cache = WeatherCache()
    weather_cache.load()

    forecast_frame = weather_cache.get_forecast_frame(problem.observation_period.begin, problem.observation_period.length)
    observation_frame = weather_cache.get_observation_frame(problem.observation_period.begin, problem.observation_period.length)
    min_time = max(forecast_frame.index.min(), observation_frame.index.min())
    max_time = min(forecast_frame.index.max(), observation_frame.index.max())

    if problem.observation_period.begin < min_time or problem.observation_period.end > max_time:
        warnings.warn('Problem observation period is reduced from [{0}, {1}] to [{2}, {3}]'.format(problem.observation_period.begin,
                                                                                                   problem.observation_period.end,
                                                                                                   min_time,
                                                                                                   max_time))

    updated_problem = copy.deepcopy(problem)
    updated_problem.trim_observation_period(TimePeriod(min_time, max_time))
    updated_problem.add_forecast('forecast', forecast_frame[(forecast_frame.index >= min_time) & (forecast_frame.index <= max_time)])
    updated_problem.add_forecast('real', observation_frame[(observation_frame.index >= min_time) & (observation_frame.index <= max_time)])

    with open(output_file, 'w') as output_file:
        json.dump(updated_problem.json_object, output_file)


def plot_forecast_command(args):
    from_date_time = getattr(args, 'from')
    to_date_time = getattr(args, 'to')

    weather_cache = WeatherCache()
    weather_cache.load()

    duration = to_date_time - from_date_time

    time_points = weather_cache.forecast_frame['DateTime'].unique()
    time_points = list({datetime.datetime.combine(pandas.to_datetime(date_time).date(), datetime.time()) for date_time in time_points})
    time_points.sort()

    diff_frames = []
    for time_point in time_points:
        forecast_frame = weather_cache.get_forecast_frame(time_point, duration)
        weather_frame = weather_cache.get_observation_frame(time_point, duration)
        diff_frame = forecast_frame - weather_frame
        diff_frame.dropna(inplace=True)
        diff_frame['Delay'] = diff_frame.index - time_point
        diff_frames.append(diff_frame)

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
        y1 = forecast_frame[city] + 2.5 * std_error_frame[city].values
        y2 = forecast_frame[city] - 2.5 * std_error_frame[city].values
        ax.fill_between(x, y1.values, y2.values, color=matplotlib.colors.CSS4_COLORS['lightgrey'], alpha=0.5)
        ax.plot(x, y1, ls='--', color=matplotlib.colors.CSS4_COLORS['grey'])
        ax.plot(x, y2, ls='--', color=matplotlib.colors.CSS4_COLORS['grey'], label='95% Confidence Interval')
        ax.plot(forecast_frame[city], label='Forecast 5-days')
        ax.plot(observation_frame[city], label='Observation')
        ax.legend()
        ax.set_title(city.name)
        ax.set_xlabel('Time')
        ax.xaxis.set_tick_params(rotation=90)
        ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(date_axis_formatter))
        ax.set_ylabel('Cloud Cover [%]')
        figure.tight_layout()

        matplotlib.pyplot.savefig('forecast_acc_{0}_{1}_{2}.png'.format(city.name, from_date_time.date(), to_date_time.date()))


def generate_command(args):
    GENERATE_PROGRAM_PATH = '/home/pmateusz/dev/quake/cmake-build-debug/quake-generate'

    problem_prefix_arg = getattr(args, 'problem_prefix')
    from_date_arg = datetime.datetime.strptime(getattr(args, 'from'), DATE_FORMAT)
    to_date_arg = datetime.datetime.strptime(getattr(args, 'to'), DATE_FORMAT)

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

        args = argparse.Namespace(**{'problem_file': temp_problem, 'output': problem})
        extend_problem_definition(args)

        if not os.path.exists(problem):
            raise Exception('Failed to generate problem {0}'.format(problem))

        os.remove(temp_problem)


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
    elif command == PLOT_FORECAST_COMMAND:
        plot_forecast_command(args)
