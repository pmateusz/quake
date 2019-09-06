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
import pickle
import json
import logging
import os
import random
import subprocess
import warnings

import scipy.stats
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
import quake.weather.scenarios

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
    extend_parser.add_argument('--scenario-generator',
                               choices=quake.weather.scenarios.ScenarioGeneratorFactory.MODEL_NAMES, required=True)

    generate_parser = sub_parsers.add_parser(GENERATE_COMMAND)
    generate_parser.add_argument('--from', action=ParseDateAction)
    generate_parser.add_argument('--to', action=ParseDateAction)
    generate_parser.add_argument('--problem-prefix')
    generate_parser.add_argument('--num-scenarios', default=0, type=int)
    generate_parser.add_argument('--scenario-generator',
                                 choices=quake.weather.scenarios.ScenarioGeneratorFactory.MODEL_NAMES, required=True)

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
                   for value in forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))][
                       'DateTime'].sort_values().unique()]
    locations = forecast_frame[(forecast_frame['Delay'] == datetime.timedelta(seconds=0))][
        'City'].sort_values().unique()

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
        forecast_frame = weather_cache.forecast_frame.loc[
                         pandas.IndexSlice[sample_start:sample_end, weather_cache.ZERO_TIME_DELTA], :].copy()
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
            stderr_mean = result.resid.mean()
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
            stderr = pandas.DataFrame(data=numpy.zeros((len(rows_index), len(cities))), columns=cities,
                                      index=rows_index)
            stderr_residuals = {city: 0.0 for city in cities}
            stderr_mean = {city: 0.0 for city in cities}
            corr = pandas.DataFrame(data=numpy.identity(len(cities)), columns=cities, index=cities)

        model_data = {}
        for station in cities:
            station_data = dict()
            station_data['residual'] = {'value': stderr_mean[station], 'stderr': stderr_residuals[station]}
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

    forecast_frame = weather_cache.get_closest_forecast_frame(problem.observation_period.begin,
                                                              problem.observation_period.length)
    real_frame = weather_cache.get_observation_frame(problem.observation_period.begin,
                                                     problem.observation_period.length)

    forecast_frame = weather_cache.fill_missing_values(forecast_frame)
    real_frame = weather_cache.fill_missing_values(real_frame)

    min_time = max(forecast_frame.index.min(), real_frame.index.min())
    max_time = min(forecast_frame.index.max(), real_frame.index.max())

    if problem.observation_period.begin < min_time or problem.observation_period.end > max_time:
        assert problem.observation_period.begin <= min_time
        assert problem.observation_period.end >= max_time
        total_reduction = (min_time - problem.observation_period.begin) + (problem.observation_period.end - max_time)
        if total_reduction > datetime.timedelta(hours=3):
            warnings.warn('Problem observation period is reduced from [{0}, {1}] to [{2}, {3}]'.format(
                problem.observation_period.begin,
                problem.observation_period.end,
                min_time,
                max_time))

    std_frame = weather_cache.get_std_frame()
    assert len(std_frame) >= len(forecast_frame)
    if len(std_frame) > len(forecast_frame):
        std_frame = std_frame.iloc[:len(forecast_frame)]

    model_factory = quake.weather.scenarios.ScenarioGeneratorFactory()

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

    generation_model = model_factory.create_model(
        quake.weather.scenarios.ScenarioGeneratorFactory.COREGIONALIZATION_MODEL_NAME, weather_cache,
        forecast_frame)
    var_model = __generate_var_model(weather_cache, min_time, datetime.timedelta(days=28))
    fitted_frame = generation_model.optimize()
    for station in fitted_frame['City'].unique():
        city_frame = fitted_frame[fitted_frame['City'] == station].copy()
        city_frame.set_index('DateTime', inplace=True)
        city_frame.drop_duplicates(inplace=True)
        var_model[station.name]['lower'] \
            = city_frame[forecast_frame.index.min():forecast_frame.index.max()]['yhat_lower'].apply(
            value_range).values.tolist()
        var_model[station.name]['upper'] \
            = city_frame[forecast_frame.index.min():forecast_frame.index.max()]['yhat_upper'].apply(
            value_range).values.tolist()
        var_model[station.name]['stderr'] = std_frame[station].values.tolist()
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
    coregionalized_observe = weather_cache.get_observation_frame(coregionalized_observation_start,
                                                                 coregionalized_observation_length)

    assert not coregionalized_observe.empty, 'No historical observations are available for coregionalization model'

    coregionalized_model = quake.weather.scenarios.CoregionalizationModel(coregionalized_observe, forecast_frame)
    coregionalized_model.optimize()

    past_error_model = quake.weather.scenarios.PastErrorsModel(weather_cache, forecast_frame)
    past_error_model.optimize()

    independent_noise_error_model = quake.weather.scenarios.HeteroscedasticIndependentNoiseModel(weather_cache,
                                                                                                 forecast_frame)
    independent_noise_error_model.optimize()

    autocorrelated_noise_error_model = quake.weather.scenarios.HeteroscedasticAutoCorrelatedNoiseModel(weather_cache,
                                                                                                       forecast_frame)
    autocorrelated_noise_error_model.optimize()

    def domain_filter(values):
        return [max(0, min(100, value)) for value in values]

    models = [past_error_model, independent_noise_error_model, autocorrelated_noise_error_model, coregionalized_model]
    model_labels = ['Past Errors Replication', 'Independent Noise Generation', 'Autocorrelated Noise Genration',
                    'Coregionalization Model']

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
                model_handle = \
                    local_ax.plot(city_series['DateTime'], domain_filter(city_series['y'].values), c='blue', alpha=0.3)[
                        0]
            model_handles.append(model_handle)
            real_handle = local_ax.plot(observation_frame[city].index.values, observation_frame[city].values, c='red')[
                0]
            forecast_handle = local_ax.plot(forecast_frame[city].index.values, forecast_frame[city].values, c='black')[
                0]

            local_ax.annotate(model_labels[model_index],
                              xy=(0.0, 0.0),
                              xycoords='axes fraction',
                              horizontalalignment='left',
                              verticalalignment='bottom',
                              bbox=BBOX_STYLE.copy())

        ax[int(len(models) / 2)].set_ylabel('Cloud Cover [%]')
        ax[-1].legend((model_handles[0], forecast_handle, real_handle),
                      ('Generated Scenario', 'Forecast Scenario', 'Real Scenario'),
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
    std_error_frame = weather_cache.get_std_frame()

    def date_axis_formatter(x, pos):
        date_time = pandas.Timestamp.fromordinal(x.astype(numpy.int64))
        return date_time.date()

    forecast_frame = weather_cache.get_forecast_frame(from_date_time, duration)
    observation_frame = weather_cache.get_observation_frame(from_date_time, duration)
    cities = [column for column in std_error_frame.columns if isinstance(column, quake.city.City)]
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

        save_figure(figure, '{0}_{1}_{2}_{3}'.format(output_prefix, city.name, simple_date_str(from_date_time),
                                                     simple_date_str(to_date_time)))


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
            observation_handle = ax[city_index].scatter(observation_frame[city].index, observation_frame[city],
                                                        marker='s', s=1,
                                                        color=OBSERVATION_COLOR)
        forecast_handle = ax[city_index].scatter(forecast_frame[city].index, forecast_frame[city], marker='s', s=1,
                                                 color=FORECAST_COLOR)
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

    model = quake.weather.scenarios.CoregionalizationModel(observation_frame, forecast_frame)
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
    with concurrent.futures.thread.ThreadPoolExecutor() as executor:
        load_problem_futures = {executor.submit(load_problem, problem_file): problem_file for problem_file in
                                problem_files}
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
            logging.warning('Failed to find a source problem for solution {0} with scenario generator {1}'.format(
                solution.observation_period,
                solution.scenario_generator))

        solution_frames.append(evaluate_scenario(source_problem.get_scenario('real'), 'out_of_sample'))
        solution_frames.append(evaluate_scenario(source_problem.get_scenario('forecast'), 'in_sample'))
    master_solution_frame = pandas.concat(solution_frames)

    worksheet_name = 'solutions'
    with pandas.ExcelWriter(output_path, engine='xlsxwriter') as writer:
        master_solution_frame.to_excel(writer, worksheet_name)
        writer.save()


class SampleSpace:
    NUM_OBSERVATIONS = 40

    class CachingSampler:
        def __init__(self, mean, covariance, pool_size=100000):
            self.__mean = mean
            self.__covariance = covariance
            self.__pool_size = pool_size

            self.__pool = None
            self.__counter = 0

        def __call__(self):
            if self.__pool is None or self.__counter >= self.__pool.shape[1]:
                self.__pool = numpy.random.multivariate_normal(self.__mean, self.__covariance, size=self.__pool_size).transpose()
                self.__counter = 0
            sample = self.__pool[:, self.__counter]
            self.__counter += 1
            return sample

    def __init__(self, frames: list):
        samples = [self.frame_to_sample(frame) for frame in frames]
        self.__sample_matrix = numpy.column_stack(samples)
        self.__sum_of_squares = numpy.sum(self.__sample_matrix ** 2, axis=0)
        self.__mean = numpy.zeros(self.__sample_matrix.shape[0])

        test_covariance = numpy.cov(self.__sample_matrix)
        test_mean = numpy.mean(self.__sample_matrix, axis=1)
        self.__test_normal = scipy.stats.multivariate_normal(test_mean, test_covariance, allow_singular=True)

        variances = []
        for row_index in range(self.__sample_matrix.shape[0]):
            variances.append(numpy.var(self.__sample_matrix[row_index, :]))
        self.__covariance = numpy.diag(variances)

        # variances = numpy.full(self.__sample_matrix.shape[0], 4.0)
        # self.__covariance = numpy.diag(variances)

    def generate_sample(self, num_samples):
        matrix = self.__sample_matrix
        for _ in range(num_samples):
            weights = numpy.random.uniform(0.0, 0.1, self.__sample_matrix.shape[1])
            scenario = self.__sample_matrix * weights.T
            print('here')

        return []

    def mgaussian_sample(self, num_samples):
        samples = self.__test_normal.rvs(num_samples)
        return [self.sample_to_frame(sample) for sample in samples]

    def mcmc_sample(self, num_samples, burn_in_period=5000, independent_sample_step=1000):
        generator = random.Random(0)
        sampler = SampleSpace.CachingSampler(self.__mean, self.__covariance)
        accept_counter = 0
        reject_counter = 0

        iterations = burn_in_period + independent_sample_step * num_samples
        samples = []
        current_sample = self.__sample_matrix[:, generator.randint(0, self.__sample_matrix.shape[1])]
        current_sample_probability = self.probability(current_sample)
        with tqdm.tqdm(range(iterations), leave=False) as iterator:
            for _ in iterator:
                raw_sample = sampler()
                candidate_sample = current_sample + raw_sample
                candidate_sample_probability = self.probability(candidate_sample)
                probability_ratio = candidate_sample_probability / current_sample_probability

                accept_threshold = min(1.0, probability_ratio)
                if accept_threshold < 1.0:
                    # consider rejection
                    test_result = generator.uniform(0.0, 1.0)
                    if test_result > accept_threshold:
                        # rejection - continue using the same sample
                        reject_counter += 1
                        samples.append(current_sample)
                        continue
                # must accept
                accept_counter += 1
                current_sample = candidate_sample
                current_sample_probability = candidate_sample_probability
                samples.append(current_sample)

        logging.info('Metropolis accept vs. reject ration: (%d, %d)', accept_counter, reject_counter)

        shortlisted_samples = samples[burn_in_period:]
        shortlisted_samples = shortlisted_samples[::independent_sample_step]

        assert len(shortlisted_samples) == num_samples

        return [self.sample_to_frame(sample) for sample in shortlisted_samples]

    def probability(self, sample: numpy.array) -> float:
        max_value = numpy.max(sample)
        if max_value > 100.0:
            return 0.0

        min_value = numpy.min(sample)
        if min_value < -100.0:
            return 0.0

        return 1.0 / (self.distance(sample) + 1.0)

    def distance(self, sample: numpy.array):
        return numpy.sum(-2 * numpy.dot(sample, self.__sample_matrix)
                         + numpy.sum(sample ** 2, axis=0)
                         + self.__sum_of_squares)

    @property
    def data(self):
        frames = []

        for sample_index in range(self.__sample_matrix.shape[1]):
            frame = self.sample_to_frame(self.__sample_matrix[:, sample_index])
            frames.append(frame)

        return frames

    @property
    def samples(self):
        return self.__sample_matrix

    def __slow_distance(self, sample: numpy.array):
        distances = []
        for sample_index in range(self.__sample_matrix.shape[1]):
            distances.append(numpy.sum((self.__sample_matrix[:, sample_index] - sample) ** 4))
        return numpy.sum(distances)

    @staticmethod
    def frame_to_sample(frame: pandas.DataFrame) -> numpy.array:
        if not (frame.index.size == SampleSpace.NUM_OBSERVATIONS and frame.index.inferred_freq == '3H' and not frame.index.has_duplicates):
            raise ValueError('Frame may have missing values')

        sample = []
        for city in quake.city.ALL:
            sample.extend(frame[city].values)

        return numpy.array(sample)

    @staticmethod
    def sample_to_frame(sample: numpy.array) -> pandas.DataFrame:
        current_delay = datetime.timedelta()
        delay_step = datetime.timedelta(hours=3)
        rows = []
        index = []
        for offset in range(SampleSpace.NUM_OBSERVATIONS):
            row = sample[offset::SampleSpace.NUM_OBSERVATIONS].flatten()

            assert len(row) == len(quake.city.ALL)

            rows.append(row)
            index.append(current_delay)

            current_delay += delay_step

        assert current_delay == datetime.timedelta(hours=120)

        frame = pandas.DataFrame(index=index, data=rows, columns=quake.city.ALL)

        return frame


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

    if os.path.exists('sample.pickle'):
        with open('sample.pickle', 'rb') as file_stream:
            filter_frames = pickle.load(file_stream)
    else:
        weather_cache = quake.weather.cache.WeatherCache()
        weather_cache.load()
        error_frames = weather_cache.get_error_frames(weather_cache.get_forecast_time_points())
        filter_frames = [error_frame for error_frame in error_frames if len(error_frame) == 40]
        with open('sample.pickle', 'wb') as file_stream:
            pickle.dump(filter_frames, file_stream)

    sample_space = SampleSpace(filter_frames)
    new_samples = sample_space.generate_sample(25)


    def draw_joint_distribution(city_x, city_y, delay, samples):
        fig, ax = matplotlib.pyplot.subplots(1, 1)

        error_x = []
        error_y = []

        for sample in samples:
            error_x.append(sample.loc[delay][city_x])
            error_y.append(sample.loc[delay][city_y])

        ax.scatter(error_x, error_y)

        return fig


    def draw_marginal_distribution(city, delay, samples):
        fig, ax = matplotlib.pyplot.subplots(1, 1)

        x = []
        for sample in samples:
            x.append(sample.loc[delay][city])
        ax.hist(x)

        return fig


    city_x = quake.city.LONDON
    city_y = quake.city.CAMBRIDGE

    data = sample_space.data

    # for hour_delay in tqdm.tqdm(range(0, 12, 3), desc='Plotting'):
    #     current_delay = datetime.timedelta(hours=hour_delay)
    #     data_figure = draw_joint_distribution(city_x, city_y, current_delay, data)
    #     data_figure.savefig('slice_joint_dist_{0}_{1}_H{2}_data.png'.format(city_x.name, city_y.name, hour_delay))
    #     matplotlib.pyplot.close(data_figure)

    # marginal_data_figure = draw_marginal_distribution(city_x, current_delay, data)
    # marginal_data_figure.savefig('marginal_dist_{0}_H{1}_sample.png'.format(city_x.name, hour_delay))
    # matplotlib.pyplot.close(marginal_data_figure)

    # sample_figure = draw_joint_distribution(city_x, city_y, current_delay, new_samples)
    # sample_figure.savefig('slice_joint_dist_{0}_{1}_H{2}_sample.png'.format(city_x.name, city_y.name, hour_delay))
    # matplotlib.pyplot.close(sample_figure)

    print('here')


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
