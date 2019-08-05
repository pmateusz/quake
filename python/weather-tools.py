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
import statsmodels.tsa.api
import tabulate
import tqdm

import quake.city
import quake.weather.cache
import quake.weather.metadata
import quake.weather.problem
import quake.weather.solution
import quake.weather.time_period

BUILD_CACHE_COMMAND = 'build-cache'
PLOT_COMMAND = 'plot-cloud-cover'
COVARIANCE_COMMAND = 'compute-covariance'
PLOT_FORECAST_COMMAND = 'plot-forecast'
PLOT_COREGIONALIZATION_COMMAND = 'plot-coregionalization'
PLOT_GENERATED_SCENARIOS_COMMAND = 'plot-generated-scenarios'
ANALYZE_COMMAND = 'analyze'
VAR_COMMAND = 'compute-var'
EXTEND_COMMAND = 'extend'
GENERATE_COMMAND = 'generate'

BBOX_STYLE = {'boxstyle': 'square,pad=0.0', 'lw': 0, 'fc': 'w', 'alpha': 0.8}


class JSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, quake.weather.problem.Problem):
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
        assert observation_frame.empty or observation_frame.index.max() <= forecast_frame.index.min()
        assert not forecast_frame.empty

        self.__observation_frame = observation_frame
        self.__forecast_frame = forecast_frame
        self.__model = None
        self.__X = None
        self.__Y = None

    def optimize(self):
        # prepare input
        X, Y = [], []
        observation_start_time = self.__observation_frame.index.min() if not self.__observation_frame.empty else self.__forecast_frame.index.min()

        def elapsed_hours(date_time):
            return int((date_time - observation_start_time).total_seconds() / 3600)

        cities = self.__forecast_frame.columns
        for city_index, city in enumerate(cities):
            if not self.__observation_frame.empty:
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
        kernel_1 = GPy.kern.RBF(1, lengthscale=6, ARD=True) \
                   + GPy.kern.Linear(1, active_dims=[0], ARD=True) \
                   + GPy.kern.White(1) \
                   + GPy.kern.Bias(1)
        kernel_2 = GPy.kern.Coregionalize(1, output_dim=len(cities), rank=matrix_rank)

        X_Gpy = self.__X[:, [1, 2]]
        self.__model = GPy.models.GPRegression(X_Gpy, self.__Y, kernel_1 ** kernel_2)
        self.__model.optimize(messages=True)

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

        num_forecast_rows = len(self.__forecast_frame)
        self.__city_covariance = dict()
        for city in tqdm.tqdm(self.__forecast_frame.columns, leave=False, desc='Building covariance matrices'):
            full_covariance_matrix = self.__weather_cache.get_forecast_error_covariance(city, error_frames)

            # limit covariance matrix to weather forecast
            covariance_matrix_to_use = full_covariance_matrix[0:num_forecast_rows, 0:num_forecast_rows]
            self.__city_covariance[city] = covariance_matrix_to_use

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


class ScenarioGeneratorFactory:
    PAST_ERRORS_MODEL_NAME = 'past_error_replication'
    INDEPENDENT_NOISE_MODEL_NAME = 'independent_error_simulation'
    CORRELATED_NOISE_MODEL_NAME = 'correlated_error_simulation'
    COREGIONALIZATION_MODEL_NAME = 'coregionalization'

    MODEL_NAMES = [PAST_ERRORS_MODEL_NAME, INDEPENDENT_NOISE_MODEL_NAME, CORRELATED_NOISE_MODEL_NAME, COREGIONALIZATION_MODEL_NAME]

    def create_model(self, model_name, weather_cache, forecast_frame):
        if model_name == self.PAST_ERRORS_MODEL_NAME:
            return PastErrorsModel(weather_cache, forecast_frame)
        elif model_name == self.INDEPENDENT_NOISE_MODEL_NAME:
            return HeteroscedasticIndependentNoiseModel(weather_cache, forecast_frame)
        elif model_name == self.CORRELATED_NOISE_MODEL_NAME:
            return HeteroscedasticAutoCorrelatedNoiseModel(weather_cache, forecast_frame)
        elif model_name == self.COREGIONALIZATION_MODEL_NAME:
            default_observation_length = datetime.timedelta(days=14)
            observation_frame \
                = weather_cache.get_observation_frame(forecast_frame.index.min() - default_observation_length, default_observation_length)
            return CoregionalizationModel(observation_frame, forecast_frame)
        assert False, 'Model name \"{0}\" is invalid' % model_name


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
    extend_parser.add_argument('--scenario-generator', choices=ScenarioGeneratorFactory.MODEL_NAMES, required=True)

    generate_parser = sub_parsers.add_parser(GENERATE_COMMAND)
    generate_parser.add_argument('--from', action=ParseDateAction)
    generate_parser.add_argument('--to', action=ParseDateAction)
    generate_parser.add_argument('--problem-prefix')
    generate_parser.add_argument('--num-scenarios', default=0, type=int)
    generate_parser.add_argument('--scenario-generator', choices=ScenarioGeneratorFactory.MODEL_NAMES, required=True)

    plot_forecast_parser = sub_parsers.add_parser(PLOT_FORECAST_COMMAND)
    plot_forecast_parser.add_argument('--from', action=ParseDateAction)
    plot_forecast_parser.add_argument('--to', action=ParseDateAction)
    plot_forecast_parser.add_argument('--output-prefix')

    compute_var = sub_parsers.add_parser(VAR_COMMAND)

    plot_coregionalization_parser = sub_parsers.add_parser(PLOT_COREGIONALIZATION_COMMAND)
    plot_coregionalization_parser.add_argument('--observation-start-time', action=ParseDateAction)
    plot_coregionalization_parser.add_argument('--forecast-start-time', action=ParseDateAction)
    plot_coregionalization_parser.add_argument('--output-prefix')

    plot_generated_scenarios_parser = sub_parsers.add_parser(PLOT_GENERATED_SCENARIOS_COMMAND)
    plot_generated_scenarios_parser.add_argument('--from', action=ParseDateAction)
    plot_generated_scenarios_parser.add_argument('--num-scenarios', default=0, type=int)
    plot_generated_scenarios_parser.add_argument('--output-prefix')

    analyze_command = sub_parsers.add_parser(ANALYZE_COMMAND)
    analyze_command.add_argument('--problem-dir', required=True)
    analyze_command.add_argument('--solution-dir', required=True)
    analyze_command.add_argument('--output', required=True)

    return parser.parse_args()


def build_cache_command(args):
    weather_cache = quake.weather.cache.WeatherCache()
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

    weather_cache = quake.weather.cache.WeatherCache()
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


# developing multi-output VAR model
def compute_vector_auto_regression(args):
    weather_cache = quake.weather.cache.WeatherCache()
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


def extend_problem_definition(args):
    problem_file = getattr(args, 'problem_file')
    output_file = getattr(args, 'output')
    num_scenarios = getattr(args, 'num_scenarios')
    scenario_generator_name = getattr(args, 'scenario_generator')

    def __generate_var_model(weather_cache, forecast_start, look_behind):
        sample_end = forecast_start
        sample_start = forecast_start - look_behind
        forecast_frame = weather_cache.forecast_frame.loc[pandas.IndexSlice[sample_start:sample_end, weather_cache.ZERO_TIME_DELTA], :].copy()
        forecast_frame.reset_index(inplace=True)
        forecast_frame.drop(columns=['Delay', 'Description'], inplace=True)
        pivot_frame = forecast_frame.pivot_table(columns=['City'], values=['CloudCover'], index=['DateTime'])
        pivot_frame.columns = pivot_frame.columns.droplevel()
        pivot_frame = weather_cache.fill_missing_values(pivot_frame)

        cities = [column for column in pivot_frame.columns if isinstance(column, quake.city.City)]
        if len(pivot_frame) > 1:
            model = statsmodels.tsa.api.VAR(pivot_frame)
            result = model.fit()
            params = result.params
            stderr = result.stderr
            stderr_residuals = result.resid.std()
            corr = pivot_frame.corr()
        else:
            rows_index = ['const']
            params_data = [numpy.zeros(len(cities)).tolist()]

            for city_index, city in enumerate(cities):
                rows_index.append('L1.{0}'.format(city.name))
                zeros = numpy.zeros(len(cities)).tolist()
                zeros[city_index] = 1.0
                params_data.append(zeros)
            params = pandas.DataFrame(data=params_data, columns=cities, index=rows_index)
            del params_data
            stderr = pandas.DataFrame(data=numpy.zeros((len(rows_index), len(cities))), columns=cities, index=rows_index)
            stderr_residuals = {city: 0.0 for city in cities}
            corr = pandas.DataFrame(data=numpy.identity(len(cities)), columns=cities, index=cities)

        model_data = {}
        for station in cities:
            station_data = dict()
            station_data['residual'] = {'value': 0, 'stderr': stderr_residuals[station]}
            station_params = params[station]
            station_stderr = stderr[station]
            station_corr = corr[station]

            for index_value in station_params.index:
                terms = index_value.split('.')
                if len(terms) == 1:
                    assert terms[0] == 'const'
                    station_data['const'] = {'value': station_params[index_value],
                                             'stderr': station_stderr[index_value]}
                elif len(terms) == 2:
                    assert terms[0] == 'L1'
                    other_station_name = terms[1]
                    other_station = quake.city.from_name(terms[1])
                    station_data[other_station_name] = {'L1': station_params[index_value],
                                                        'L1.stderr': station_stderr[index_value],
                                                        'corr': station_corr[other_station]}
                else:
                    raise RuntimeError('Invalid number of terms')
            model_data[station.name] = station_data
        return model_data

    with open(problem_file, 'r') as input_stream:
        json_object = json.load(input_stream)
        problem = quake.weather.problem.Problem(json_object)

    weather_cache = quake.weather.cache.WeatherCache()
    weather_cache.load()

    forecast_frame = weather_cache.get_closest_forecast_frame(problem.observation_period.begin, problem.observation_period.length)
    real_frame = weather_cache.get_observation_frame(problem.observation_period.begin, problem.observation_period.length)

    forecast_frame = weather_cache.fill_missing_values(forecast_frame)
    real_frame = weather_cache.fill_missing_values(real_frame)

    min_time = max(forecast_frame.index.min(), real_frame.index.min())
    max_time = min(forecast_frame.index.max(), real_frame.index.max())

    if problem.observation_period.begin < min_time or problem.observation_period.end > max_time:
        assert problem.observation_period.begin <= min_time
        assert problem.observation_period.end >= max_time
        total_reduction = (min_time - problem.observation_period.begin) + (problem.observation_period.end - max_time)
        if total_reduction > datetime.timedelta(hours=3):
            warnings.warn('Problem observation period is reduced from [{0}, {1}] to [{2}, {3}]'.format(problem.observation_period.begin,
                                                                                                       problem.observation_period.end,
                                                                                                       min_time,
                                                                                                       max_time))

    model_factory = ScenarioGeneratorFactory()

    def value_range(value):
        return max(0.0, min(100.0, value))

    def enforce_cloud_cover_limits(frame):
        frame_to_use = frame.copy()
        for column in frame.columns:
            frame_to_use[column] = frame_to_use[column].apply(value_range)
        return frame_to_use

    scenario_frames = []
    if num_scenarios > 0:
        generation_model = model_factory.create_model(scenario_generator_name, weather_cache, forecast_frame)
        generation_model.optimize()

        sample_frames = generation_model.samples(num_scenarios)
        for sample_frame in sample_frames:
            scenario_frame = sample_frame.pivot_table(index='DateTime', columns='City', values='y')
            scenario_frame = enforce_cloud_cover_limits(scenario_frame)
            scenario_frames.append(scenario_frame)

    def trim_to_time_interval(frame):
        return frame[(frame.index >= min_time) & (frame.index <= max_time)]

    updated_problem = copy.deepcopy(problem)
    updated_problem.trim_observation_period(quake.weather.time_period.TimePeriod(min_time, max_time))
    updated_problem.add_forecast('forecast', trim_to_time_interval(forecast_frame))
    updated_problem.add_forecast('real', trim_to_time_interval(real_frame))

    for scenario_index, scenario_frame in enumerate(scenario_frames):
        updated_problem.add_forecast('scenario_{0}'.format(scenario_index), trim_to_time_interval(scenario_frame))

    updated_problem.set_metadata('scenarios_number', len(scenario_frames))
    updated_problem.set_metadata('scenario_generator', scenario_generator_name)

    generation_model = model_factory.create_model(ScenarioGeneratorFactory.COREGIONALIZATION_MODEL_NAME, weather_cache, forecast_frame)
    var_model = __generate_var_model(weather_cache, min_time, datetime.timedelta(days=28))
    fitted_frame = generation_model.optimize()
    for station in fitted_frame['City'].unique():
        city_frame = fitted_frame[fitted_frame['City'] == station].copy()
        city_frame.set_index('DateTime', inplace=True)
        city_frame.drop_duplicates(inplace=True)
        var_model[station.name]['lower'] \
            = city_frame[forecast_frame.index.min():forecast_frame.index.max()]['yhat_lower'].apply(value_range).values.tolist()
        var_model[station.name]['upper'] \
            = city_frame[forecast_frame.index.min():forecast_frame.index.max()]['yhat_upper'].apply(value_range).values.tolist()
    updated_problem.set_var_model(var_model)

    with open(output_file, 'w') as output_file:
        json.dump(updated_problem.json_object, output_file)


def generate_command(args):
    GENERATE_PROGRAM_PATH = '/home/pmateusz/dev/quake/cmake-build-debug/quake-generate'

    problem_prefix_arg = getattr(args, 'problem_prefix')
    from_date_arg = getattr(args, 'from')
    to_date_arg = getattr(args, 'to')
    num_scenarios = getattr(args, 'num_scenarios')
    generation_model_name = getattr(args, 'scenario_generator')

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

        args = argparse.Namespace(**{'problem_file': temp_problem, 'output': problem,
                                     'num_scenarios': num_scenarios, 'scenario_generator': generation_model_name})
        extend_problem_definition(args)

        if not os.path.exists(problem):
            raise Exception('Failed to generate problem {0}'.format(problem))

        os.remove(temp_problem)


def plot_generated_scenarios(args):
    forecast_time = getattr(args, 'from')
    num_samples = getattr(args, 'num_scenarios')
    output_prefix = getattr(args, 'output_prefix')

    weather_cache = quake.weather.cache.WeatherCache()
    weather_cache.load()
    forecast_frame = weather_cache.get_closest_forecast_frame(forecast_time, weather_cache.forecast_length)

    coregionalized_observation_length = datetime.timedelta(days=14)
    coregionalized_observation_start = forecast_time - coregionalized_observation_length
    coregionalized_observe = weather_cache.get_observation_frame(coregionalized_observation_start, coregionalized_observation_length)

    assert not coregionalized_observe.empty, 'No historical observations are available for coregionalization model'

    coregionalized_model = CoregionalizationModel(coregionalized_observe, forecast_frame)
    coregionalized_model.optimize()

    past_error_model = PastErrorsModel(weather_cache, forecast_frame)
    past_error_model.optimize()

    independent_noise_error_model = HeteroscedasticIndependentNoiseModel(weather_cache, forecast_frame)
    independent_noise_error_model.optimize()

    autocorrelated_noise_error_model = HeteroscedasticAutoCorrelatedNoiseModel(weather_cache, forecast_frame)
    autocorrelated_noise_error_model.optimize()

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


def save_figure(fig, file_name):
    fig.savefig('{0}.png'.format(file_name))


def plot_command(args):
    from_arg = getattr(args, 'from')
    if from_arg:
        left_time_limit = datetime.datetime.strptime(from_arg, '%Y-%m-%d')
    else:
        left_time_limit = None

    weather_cache = quake.weather.cache.WeatherCache()
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

    weather_cache = quake.weather.cache.WeatherCache()
    weather_cache.load()

    duration = to_date_time - from_date_time
    time_points = weather_cache.get_forecast_time_points()
    diff_frames = weather_cache.get_error_frames(time_points)
    diff_frame = pandas.concat(diff_frames, ignore_index=True)
    del diff_frames

    delay_values = list(pandas.to_timedelta(value, unit='ns') for value in diff_frame['Delay'].unique())
    delay_values.sort()

    # error_data = []
    # for delay_value in delay_values:
    #     delay_diff_frame = diff_frame[diff_frame['Delay'] == delay_value]
    #
    #     for city_column in delay_diff_frame.columns:
    #         if not isinstance(city_column, quake.city.City):
    #             continue
    #         for error_value in delay_diff_frame[city_column]:
    #             error_data.append([city_column, pandas.Timedelta(value=delay_value.total_seconds(), unit='s'), error_value])
    # columns = ['City', 'Delay', 'Error']
    # error_frame = pandas.DataFrame(columns=columns, data=error_data)

    # delay_values = error_frame['Delay'].unique()
    rows = []
    cities = [column for column in diff_frame.columns if isinstance(column, quake.city.City)]
    for delay_value in delay_values:
        filter_frame = diff_frame[diff_frame['Delay'] == delay_value].copy()
        row = {city: filter_frame[city].std() for city in cities}
        rows.append(row)
    std_error_frame = pandas.DataFrame(index=delay_values, data=rows)

    def date_axis_formatter(x, pos):
        date_time = pandas.Timestamp.fromordinal(x.astype(numpy.int64))
        return date_time.date()

    forecast_frame = weather_cache.get_forecast_frame(from_date_time, duration)
    observation_frame = weather_cache.get_observation_frame(from_date_time, duration)
    for city in cities:
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

        save_figure(figure, '{0}_{1}_{2}_{3}'.format(output_prefix, city.name, simple_date_str(from_date_time), simple_date_str(to_date_time)))


def plot_coregionalization(args):
    forecast_start_time = getattr(args, 'forecast_start_time')
    observation_start_time = getattr(args, 'observation_start_time')
    output_prefix = getattr(args, 'output_prefix')

    weather_cache = quake.weather.cache.WeatherCache()
    weather_cache.load()

    observation_duration = forecast_start_time - observation_start_time
    forecast_frame = weather_cache.get_closest_forecast_frame(forecast_start_time, weather_cache.forecast_length)

    assert not forecast_frame.empty, 'Forecast frame is empty'

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
        if not observation_frame.empty:
            observation_handle = ax[city_index].scatter(observation_frame[city].index, observation_frame[city], marker='s', s=1,
                                                        color=OBSERVATION_COLOR)
        forecast_handle = ax[city_index].scatter(forecast_frame[city].index, forecast_frame[city], marker='s', s=1, color=FORECAST_COLOR)
        ax[city_index].set_ylim(-25, 125)

        at = matplotlib.offsetbox.AnchoredText(city.name, frameon=False, loc='lower left')
        ax[city_index].add_artist(at)

    for tick in ax[-1].get_xticklabels():
        tick.set_rotation(90)
    ax[int(len(cities) / 2)].set_ylabel('Cloud Cover [%]')
    ax[-1].set_xlabel('Date')

    if observation_handle:
        handles = [observation_handle, forecast_handle]
        handle_labels = ['Observation', 'Forecast']
    else:
        handles = [forecast_handle]
        handle_labels = ['Forecast']

    legend_artist = ax[-1].legend(handles, handle_labels, ncol=2, loc='center', bbox_to_anchor=(0.5, -2))
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

        if not observation_frame.empty:
            observation_city_series = observation_frame[city]
            observation_handle = ax[city_index].scatter(observation_city_series.index, observation_city_series.values,
                                                        marker='s', s=1.0, color=OBSERVATION_COLOR)

        forecast_city_series = forecast_frame[city]
        forecast_handle = ax[city_index].scatter(forecast_city_series.index, forecast_city_series.values,
                                                 marker='s', s=1.0, color=FORECAST_COLOR)
    for tick in ax[-1].get_xticklabels():
        tick.set_rotation(90)
    ax[int(len(cities) / 2)].set_ylabel('Cloud Cover [%]')
    ax[-1].set_xlabel('Date')

    if not observation_frame.empty:
        handles = [mean_handle, confidence_handle, observation_handle, forecast_handle,
                   sample_handles[0], sample_handles[1], sample_handles[2], sample_handles[3]]
        labels = ['Mean', 'Confidence', 'Observation', 'Forecast', 'Sample 1', 'Sample 2', 'Sample 3', 'Sample 4']
    else:
        handles = [mean_handle, confidence_handle, forecast_handle,
                   sample_handles[0], sample_handles[1], sample_handles[2], sample_handles[3]]
        labels = ['Mean', 'Confidence', 'Forecast', 'Sample 1', 'Sample 2', 'Sample 3', 'Sample 4']

    legend_artist = ax[-1].legend(handles, labels, ncol=4, loc='center', bbox_to_anchor=(0.5, -2.1))
    ax[-1].add_artist(legend_artist)
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.2, hspace=0.10)

    save_figure(fig, '{0}_{1}_output'.format(output_prefix, simple_date_str(forecast_start_time)))
    matplotlib.pyplot.close(fig)


def load_problem(file_path):
    with open(file_path, 'r') as input_stream:
        problem_json = json.load(input_stream)
        problem = quake.weather.problem.Problem(problem_json)
        return problem


def analyze_command(args):
    problem_dir = getattr(args, 'problem_dir')
    solution_dir = getattr(args, 'solution_dir')
    output_path = getattr(args, 'output')

    def is_json_file(file_path):
        if not os.path.isfile(file_path):
            return False

        _, ext = os.path.splitext(file_path)
        return ext == '.json'

    def format_time_delta(value: datetime.timedelta) -> str:
        remaining_sec = int(value.total_seconds())
        hours = remaining_sec // 3600
        remaining_sec -= 3600 * hours

        minutes = remaining_sec // 60
        remaining_sec -= 60 * minutes

        return "{0:02d}:{1:02d}:{2:02d}".format(hours, minutes, remaining_sec)

    def format_time(value: datetime.datetime) -> str:
        return value.strftime('%Y-%b-%d %H:%M:%S')

    def evaluate_scenario(scenario, evaluation_type):
        data = []
        for station in solution.stations:
            station_keys_transferred = 0.0
            for observation in solution.observations(station):
                station_keys_transferred += source_problem.get_transferred_keys(station, observation, scenario)
            station_transfer_share = source_problem.transfer_share(station)
            data.append({'city': station,
                         'keys_transferred': station_keys_transferred,
                         'transfer_share': station_transfer_share,
                         'traffic_index': station_keys_transferred / station_transfer_share})
        solution_frame = pandas.DataFrame(data=data)
        solution_frame['observation_start'] = format_time(solution.observation_period.begin)
        solution_frame['observation_end'] = format_time(solution.observation_period.end)
        solution_frame[quake.weather.metadata.SCENARIO_GENERATOR] = solution.scenario_generator
        solution_frame[quake.weather.metadata.SOLUTION_TYPE] = solution.solution_type
        solution_frame[quake.weather.metadata.GAP] = solution.gap
        solution_frame[quake.weather.metadata.GAP_LIMIT] = solution.gap_limit
        solution_frame[quake.weather.metadata.TIME_LIMIT] = solution.time_limit
        solution_frame[quake.weather.metadata.INTERVAL_STEP] = format_time_delta(solution.interval_step)
        solution_frame[quake.weather.metadata.SCENARIOS_NUMBER] = solution.scenarios_number
        solution_frame[quake.weather.metadata.SOLUTION_METHOD] = solution.solution_method
        solution_frame[quake.weather.metadata.TARGET_TRAFFIC_INDEX] = solution.target_traffic_index
        solution_frame['epsilon'] = solution.epsilon
        solution_frame['solution_id'] = solution_index
        solution_frame['evaluation'] = evaluation_type
        return solution_frame

    solutions = []
    solution_files = [os.path.join(solution_dir, solution_file) for solution_file in os.listdir(solution_dir)]
    solution_files = list(filter(is_json_file, solution_files))
    for solution_file_path in tqdm.tqdm(solution_files, desc='Loading Solution Files', leave=False):
        with open(solution_file_path, 'r') as input_stream:
            solution_json = json.load(input_stream)
            solution = quake.weather.solution.Solution(solution_json)
            solutions.append(solution)

    problems = []
    problem_files = [os.path.join(problem_dir, problem_file) for problem_file in os.listdir(problem_dir)]
    problem_files = list(filter(is_json_file, problem_files))
    with concurrent.futures.process.ProcessPoolExecutor() as executor:
        load_problem_futures = {executor.submit(load_problem, problem_file): problem_file for problem_file in problem_files}
        for future in tqdm.tqdm(concurrent.futures.as_completed(load_problem_futures),
                                total=len(load_problem_futures), desc='Loading Problem Files', leave=False):
            try:
                problems.append(future.result())
            except Exception as ex:
                problem_file_path = load_problem_futures[future]
                logging.exception('Attempt to load the problem file %s generated error %s', problem_file_path, ex)

    solution_frames = []
    for solution_index, solution in tqdm.tqdm(enumerate(solutions), desc='Analyzing Solutions', leave=False):
        source_problem = None
        for problem in problems:
            if problem.observation_period == solution.observation_period and problem.scenario_generator == solution.scenario_generator:
                source_problem = problem
                break

        if not source_problem:
            logging.warning('Failed to find a source problem for solution {0} with scenario generator {1}'.format(solution.observation_period,
                                                                                                                  solution.scenario_generator))

        solution_frames.append(evaluate_scenario(source_problem.get_scenario('real'), 'out_of_sample'))
        solution_frames.append(evaluate_scenario(source_problem.get_scenario('forecast'), 'in_sample'))
    master_solution_frame = pandas.concat(solution_frames)

    worksheet_name = 'solutions'
    with pandas.ExcelWriter(output_path, engine='xlsxwriter') as writer:
        master_solution_frame.to_excel(writer, worksheet_name)
        writer.save()


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
    elif command == PLOT_COREGIONALIZATION_COMMAND:
        plot_coregionalization(args)
    elif command == PLOT_GENERATED_SCENARIOS_COMMAND:
        plot_generated_scenarios(args)
    elif command == ANALYZE_COMMAND:
        analyze_command(args)


    def plot_errors(station, begin_start_time, end_start_time):
        weather_cache = quake.weather.cache.WeatherCache()
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
        weather_cache = quake.weather.cache.WeatherCache()
        weather_cache.load()

        forecast_frame = weather_cache.get_forecast_frame(start_time, duration)
        observation_frame = weather_cache.get_observation_frame(start_time, duration)
        error_frame = forecast_frame - observation_frame
        error_frame.dropna(inplace=True)
        error_frame.reset_index(inplace=True)

        import statsmodels.graphics.tsaplots
        statsmodels.graphics.tsaplots.plot_pacf(error_frame[quake.city.LONDON].values.transpose())
        matplotlib.pyplot.show(block=True)


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

    # def test_error_regression_md():
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
        weather_cache = quake.weather.cache.WeatherCache()
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
                # # variance stabilising transform
                # if std != 0.0:
                #     Y.append((value - mean) / std)
                # else:
                #     Y.append((value - mean))
                Y.append(value)
                X.append(delay)
            Y_mean.append([mean])
            Y_std.append([std if std > 0.0 else 1.0])

        # Y_mean = numpy.array(Y_mean)
        # Y_std = numpy.array(Y_std)

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


    weather_cache = quake.weather.cache.WeatherCache()
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

    # time_points = [pandas.to_datetime(value)
    #                for value in forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))]['DateTime'].sort_values().unique()]
    # locations = forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))]['City'].sort_values().unique()
    #
    # for time_point in time_points:
    #     local_forecast_sample = forecast_sample(forecast_frame, time_point)
    #     local_observation_sample = observation_sample(forecast_frame, time_point)
    #     for location in locations:
    #         residual_series = local_forecast_sample[location] - local_observation_sample[location]
    #         residual_frame = residual_series.to_frame()
    #         residual_frame['DateTime'] = pandas.to_datetime(residual_frame.index)
    #         min_time = residual_frame['DateTime'].min()
    #         residual_frame['Delay'] = residual_frame['DateTime'] - min_time
    #         pass
    #
    # result_frame = pivot_transform(forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))
    #                                               & (forecast_frame['DateTime'] >= datetime.datetime(2019, 6, 20))
    #                                               & (forecast_frame['DateTime'] <= datetime.datetime(2019, 7, 12))])
    #
