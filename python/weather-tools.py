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
import concurrent.futures
import copy
import datetime
import json
import logging
import os
import subprocess
import warnings

import matplotlib.dates
import matplotlib.pyplot
import matplotlib.ticker
import numpy
import pandas
import quake.city
import tabulate
import tqdm

BUILD_CACHE_COMMAND = 'build-cache'
PLOT_COMMAND = 'plot-cloud-cover'
COVARIANCE_COMMAND = 'compute-covariance'
VAR_COMMAND = 'compute-var'
EXTEND_COMMAND = 'extend'
GENERATE_COMMAND = 'generate'


class WeatherCache:
    FILE_TABLE = 'a'
    FILE_MODE = 'w'
    FORECAST_CACHE_FILE = 'forecast.hdf'
    OBSERVATION_CACHE_FILE = 'observation.hdf'

    def __init__(self):
        self.__forecast_frame = None
        self.__observation_frame = None

    def rebuild(self):
        resolved_root_directory = os.path.expanduser('~/OneDrive/dev/quake/data/forecasts/')

        file_paths = [os.path.abspath(os.path.join(resolved_root_directory, file_name))
                      for file_name in os.listdir(resolved_root_directory) if file_name.endswith('.json')]

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

        observation_frame = pandas.read_csv('/home/pmateusz/dev/quake/data/weather/ground_station_data_set_filled.csv')
        observation_frame['date_time'] = observation_frame['dt_iso'].apply(
            lambda date_string: datetime.datetime.strptime(date_string, '%Y-%m-%d %H:%M:%S %z %Z'))
        observation_frame.drop(columns=['weather_id', 'weather_icon', 'dt', 'dt_iso',
                                        'lat', 'lon',
                                        'rain_today', 'snow_today',
                                        'rain_1h', 'snow_1h',
                                        'rain_3h', 'snow_3h',
                                        'rain_24h', 'snow_24h'], inplace=True)
        observation_frame['city_name'] = observation_frame['city_id'].apply(quake.city.from_key)

        self.__forecast_frame = forecast_frame
        self.__observation_frame = observation_frame

    def load(self):
        self.__forecast_frame = pandas.read_hdf(self.FORECAST_CACHE_FILE, self.FILE_TABLE)
        self.__observation_frame = pandas.read_hdf(self.OBSERVATION_CACHE_FILE, self.FILE_TABLE)

    def save(self):
        self.__forecast_frame.to_hdf(self.FORECAST_CACHE_FILE, self.FILE_TABLE, mode=self.FILE_MODE)
        self.__observation_frame.to_hdf(self.OBSERVATION_CACHE_FILE, self.FILE_TABLE, mode=self.FILE_MODE)

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
        index = frame['DateTime'] + frame['Delay']
        index.drop_duplicates(inplace=True)
        index.sort_values(inplace=True)
        index_to_position = {index_value: index_position for index_position, index_value in enumerate(index)}

        station_data = []
        cities = frame['City'].unique()
        for city in cities:
            city_frame = frame[frame['City'] == city].copy()
            city_frame['IndexDateTime'] = city_frame['DateTime'] + city_frame['Delay']
            city_frame.set_index('IndexDateTime', inplace=True)
            city_frame.sort_index(inplace=True)

            cloud_cover_array = [0] * len(index)
            for row in city_frame.itertuples():
                cloud_cover_array[index_to_position[row.Index]] = row.CloudCover
            station_data.append({'station': city.name, 'cloud_cover': cloud_cover_array})

        index_values = [value.strftime(TimePeriod.DATETIME_FORMAT) for value in index]
        return {'index': index_values, 'stations': station_data}


class JSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Problem):
            return obj.json_object
        if isinstance(obj, quake.city.City):
            return obj.name
        return json.JSONEncoder.default(self, obj)


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
    _problem_file = getattr(args, 'problem_file')
    _output_file = getattr(args, 'output')

    with open(_problem_file, 'r') as _input_stream:
        _json_object = json.load(_input_stream)
        _problem = Problem(_json_object)

    _weather_cache = WeatherCache()
    _weather_cache.load()

    def extract_forecast(observation_period: TimePeriod, weather_cache: WeatherCache):
        forecast_frame = weather_cache.forecast_frame.copy()
        forecast_frame['ForecastDateTime'] = forecast_frame['DateTime'] - forecast_frame['Delay']

        forecast_date_time = forecast_frame[forecast_frame.apply(lambda row: row['ForecastDateTime'].date() == observation_period.begin.date(), axis=1)]['DateTime'].min()
        forecast_frame_to_use = forecast_frame[forecast_frame['ForecastDateTime'] == forecast_date_time].copy()
        # available_forecast_end_time = available_forecast_start_time + forecast_frame_to_use['Delay'].max()
        # forecast_frame_to_use['EffectiveDateTime'] = forecast_frame_to_use['DateTime'] + forecast_frame_to_use['Delay']
        # if available_forecast_end_time < observation_period[1]:
        #     logging.fatal('Available weather forecast [%s,%s] does not cover requested observation period [%s, %s]',
        #                   available_forecast_start_time,
        #                   available_forecast_end_time,
        #                   observation_period[0],
        #                   observation_period[1])

        return forecast_frame_to_use

    _forecast_frame = extract_forecast(_problem.observation_period, _weather_cache)

    def extract_observation(observation_period: TimePeriod, weather_cache: WeatherCache):
        forecast_frame = weather_cache.forecast_frame
        filtered_frame = forecast_frame[forecast_frame['Delay'] == datetime.timedelta()]
        filtered_frame = filtered_frame[(filtered_frame['DateTime'] >= observation_period.begin)
                                        & (filtered_frame['DateTime'] <= observation_period.end)].copy()

        # available_observation_start_time = filtered_frame['DateTime'].min()
        # available_observation_end_time = filtered_frame['DateTime'].max()
        # if available_observation_end_time < observation_period[1] or available_observation_start_time > observation_period[0]:
        #     logging.fatal('Available weather observation [%s,%s] does not cover requested observation period [%s, %s]',
        #                   available_observation_start_time,
        #                   available_observation_end_time,
        #                   observation_period[0],
        #                   observation_period[1])

        return filtered_frame

    _observation_frame = extract_observation(_problem.observation_period, _weather_cache)
    _observation_time_series = list(_observation_frame['DateTime'].unique())
    _forecast_time_series = list((_forecast_frame['DateTime']).unique())
    _min_time = pandas.to_datetime(max(min(_observation_time_series), min(_forecast_time_series)))
    _max_time = pandas.to_datetime(min(max(_observation_time_series), max(_forecast_time_series)))

    _filtered_forecast_frame = _forecast_frame[(_forecast_frame['DateTime'] >= _min_time) & (_forecast_frame['DateTime'] <= _max_time)]
    _filtered_observation_frame = _observation_frame[(_observation_frame['DateTime'] >= _min_time) & (_observation_frame['DateTime'] <= _max_time)]

    _updated_problem = copy.deepcopy(_problem)
    _actual_time_period = _updated_problem.observation_period
    if _actual_time_period.begin < _min_time or _actual_time_period.end > _max_time:
        logging.warning('Observation period is reduced from [%s, %s] to [%s, %s]', _updated_problem.observation_period.begin,
                        _updated_problem.observation_period.end,
                        _min_time,
                        _max_time)
        _updated_problem.trim_observation_period(TimePeriod(_min_time, _max_time))
    _updated_problem.add_forecast('forecast', _filtered_forecast_frame)
    _updated_problem.add_forecast('real', _filtered_observation_frame)

    with open(_output_file, 'w') as _output_file:
        json.dump(_updated_problem.json_object, _output_file)


def generate_command(args):
    DATE_FORMAT = '%Y-%m-%d'
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
