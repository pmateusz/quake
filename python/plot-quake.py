#!/usr/bin/env python3
#
# Copyright 2018 Mateusz Polnik
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
import csv
import datetime
import glob
import json
import math
import operator
import os
import pathlib
import re
import statistics
import typing
import warnings

import PIL
import matplotlib.cm
import matplotlib.dates
import matplotlib.gridspec
import matplotlib.pyplot
import matplotlib.ticker
import numpy
import pandas
import tabulate
import tqdm

import quake.city
import quake.cloud_cover
import quake.minizinc
import quake.multistage_simulation
import quake.solution
import quake.sunset_sunrise
import quake.transfer_rate
import quake.weather.problem
import quake.weather.time_period

TIME_KEY = 'Time'


def split_total_seconds(value):
    hours = int(value // matplotlib.dates.SEC_PER_HOUR)
    minutes = int((value - hours * matplotlib.dates.SEC_PER_HOUR) // matplotlib.dates.SEC_PER_MIN)
    return hours, minutes


class PandasTimeDeltaConverter:

    def __call__(self, x, pos=None):
        timedelta = pandas.Timedelta(value=x, unit='ns')
        py_timedelta = timedelta.to_pytimedelta()
        hours, minutes = split_total_seconds(py_timedelta.total_seconds())
        return '{0:02d}:{1:02d}'.format(hours, minutes)


class NumpyTimeDeltaConverter:

    def __call__(self, x, pos=None):
        if isinstance(x, numpy.int64):
            timedelta = datetime.timedelta(seconds=x.item())
        else:
            timedelta = datetime.timedelta(seconds=x)
        hours, minutes = split_total_seconds(timedelta.total_seconds())
        return '{0:02d}:{1:02d}'.format(hours, minutes)


def load_elevation(file_path):
    with open(file_path, 'r') as file:
        dialect = csv.Sniffer().sniff(file.read(4096))
        dialect.delimiter = ','
        file.seek(0)
        reader_it = iter(csv.reader(file, dialect))
        columns = next(reader_it, None)
        columns_to_use = [TIME_KEY] + [name.strip() for name in columns[1:]]

        data = []
        for line in reader_it:
            values = [datetime.datetime.strptime(line[0], '%Y-%b-%d %H:%M:%S')]
            values.extend(map(lambda x: math.degrees(float(x)), line[1:]))
            data.append(values)
        data_frame = pandas.DataFrame(columns=columns_to_use, data=data)
        data_frame.set_index(TIME_KEY)
        return data_frame


def load_mini_zinc_solution(file_path):
    results = []
    begin_date_time_format = '%Y-%b-%d %H:%M:%S'
    end_date_time_format = '%Y-%b-%d %H:%M:%S.%f'
    with open(file_path, 'r') as file_stream:
        first_line = next(file_stream)
        if first_line.strip() == '#!minizinc':
            raw_start_time = next(file_stream)
            raw_start_time = raw_start_time.strip().replace('"', '')
            start_time = datetime.datetime.strptime(raw_start_time, begin_date_time_format)
            for line in file_stream:
                station, raw_period = line.split(';')
                raw_begin_period, raw_end_period = raw_period.strip().split('/')
                begin_period = start_time + datetime.timedelta(seconds=int(raw_begin_period))
                end_period = start_time + datetime.timedelta(seconds=int(raw_end_period))
                results.append((station, begin_period, end_period - begin_period))
        else:
            for line in file_stream:
                station, raw_period = line.split(';')
                period = raw_period.strip().replace('[', '').replace(']', '')
                raw_begin_period, raw_end_period = period.split('/')
                begin_period = datetime.datetime.strptime(raw_begin_period, begin_date_time_format)
                end_period = datetime.datetime.strptime(raw_end_period, end_date_time_format)
                if end_period.microsecond == 999999:
                    end_period = end_period + datetime.timedelta(microseconds=1)
                results.append((station, begin_period, end_period - begin_period))
    return results


def load_model(model_file):
    loader = quake.minizinc.MiniZincLoader()
    with open(model_file, 'r') as file_stream:
        model = loader.load(file_stream)

    start_time = datetime.datetime.strptime(model['start_time'], '%Y-%b-%d %H:%M:%S')
    key_rate = model['key_rate']
    time_offset = model['time_offset']
    stations = model['STATION']

    data = []
    for offset_index, current_offset in enumerate(time_offset):
        row = [start_time + datetime.timedelta(seconds=current_offset)]
        for station_index in range(len(stations)):
            row.append(key_rate[station_index][offset_index])
        data.append(row)

    columns = ['Time']
    columns.extend(stations)

    return pandas.DataFrame(columns=columns, data=data)


# def load_key_rate(elevation_frame, transfer_elevation_frame, configuration):
#     transfer_elevation_frame_to_use \
#         = transfer_elevation_frame.loc[transfer_elevation_frame[CONFIG_KEY] == configuration]
#     elevation_keyrate_pairs = transfer_elevation_frame_to_use[[ELEVATION_KEY, KEY_RATE_KEY]] \
#         .sort_values([ELEVATION_KEY, KEY_RATE_KEY])
#     elevation_values = list(elevation_keyrate_pairs[ELEVATION_KEY])
#     keyrate_values = list(elevation_keyrate_pairs[KEY_RATE_KEY])
#
#     data = []
#     for elevation_row in elevation_frame.values:
#         transfer_row = [elevation_row[0]]
#         for elevation in elevation_row[1:]:
#             assert -90.0 <= elevation <= 90.0
#             transfer = 0.0
#
#             if elevation >= 15.0:
#                 index = bisect.bisect(elevation_values, elevation)
#                 left_index = index - 1
#                 if 0 < index < len(elevation_values):
#                     dist_angle_left = math.fabs(elevation - elevation_values[left_index])
#                     dist_angle_right = math.fabs(elevation_values[index] - elevation)
#                     if dist_angle_left <= dist_angle_right:
#                         transfer = keyrate_values[left_index]
#                     else:
#                         transfer = keyrate_values[index]
#                 elif index == 0:
#                     transfer = keyrate_values[left_index]
#                 elif index == len(elevation_values):
#                     transfer = keyrate_values[left_index]
#                 else:
#                     raise IndexError(str(index))
#             transfer_row.append(transfer)
#         data.append(transfer_row)
#     data_frame = pandas.DataFrame(columns=elevation_frame.columns.values, data=data)
#     data_frame.set_index(TIME_KEY)
#     return data_frame


def load_solution_bundle(file_path):
    with open(file_path, 'r') as input_stream:
        document = json.load(input_stream)
        raw_bundle = quake.solution.SolutionBundle.from_json(document)
        return quake.transfer_rate.adjust_128_to_256(raw_bundle)


def format_timedelta(x, pos=None):
    time_delta = matplotlib.dates.num2timedelta(x)
    hours = int(time_delta.seconds / matplotlib.dates.SEC_PER_HOUR)
    minutes = int(time_delta.seconds / matplotlib.dates.SEC_PER_MIN) - 60 * hours
    return '{0:02d}:{1:02d}'.format(hours, minutes)


# def plot_frame(background_frame, y_label, solution_frame):
#     EXTRA_COLUMNS = {TIME_KEY, CONFIG_KEY}
#
#     min_y = min(background_frame[column].min()
#                 for column in background_frame.columns.values
#                 if column not in EXTRA_COLUMNS)
#     max_y = max(background_frame[column].max()
#                 for column in background_frame.columns.values
#                 if column not in EXTRA_COLUMNS)
#
#     stations = solution_frame.Station.unique()
#     figure, axes = matplotlib.pyplot.subplots(len(stations), 1, sharex=True)
#
#     min_x = max(solution_frame['StartTime'].min(), background_frame['Time'].min()) - datetime.timedelta(seconds=1800)
#     max_x = min(solution_frame['EndTime'].max(), background_frame['Time'].max()) + datetime.timedelta(seconds=1800)
#
#     for axis, station in zip(axes, stations):
#         axis.scatter(background_frame[TIME_KEY].values, background_frame[station].values, s=0.5)
#         axis.set_xlim(matplotlib.dates.date2num(min_x), matplotlib.dates.date2num(max_x))
#         axis.set_ylim(min_y, max_y)
#         axis.annotate(station,
#                       xy=(1.2, .5),
#                       xycoords='axes fraction',
#                       horizontalalignment='right',
#                       verticalalignment='center')
#
#         jobs_frame = solution_frame.loc[solution_frame.Station == station]
#         for row in jobs_frame.itertuples():
#             axis.axvspan(matplotlib.dates.date2num(row[2]),
#                          matplotlib.dates.date2num(row[3]),
#                          facecolor='0.2',
#                          alpha=0.5)
#
#     axes[-1].xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(format_timedelta))
#     matplotlib.pyplot.xticks(rotation=45)
#     figure.tight_layout()
#     figure.subplots_adjust(left=0.1, right=0.8, bottom=0.18, wspace=0.025, hspace=0.25)
#     figure.text(0.4, 0.04, 'Time of Day: {0} - {1}'.format(min_x.date(), max_x.date()), ha='center', va='center')
#     figure.text(0.03, 0.5, y_label, ha='center', va='center', rotation='vertical')


# def plot_mini_zinc_solution():
#     elevation_frame = load_elevation('./data/elevation/elevation_2018_09_07-08.csv')
#
#     transfer_elevation_frame = load_key_rates('./data/key_rate/data_025eff.ods')
#     transfer_frame = load_key_rate(elevation_frame, transfer_elevation_frame, '633')
#     schedule_solution = load_mini_zinc_solution('./data/solution/london_glasgow_1sec_mzn_output.txt')
#
#     plot_frame(transfer_frame, 'Key Rate', 'key_rate_plot.png')
#     plot_frame(elevation_frame, 'Elevation Angle', 'elevation_plot.png')


FORMAT = 'pdf'
FIGURE_FONT_SIZE = 12
FIGURE_WIDTH_SIZE = 7
FIGURE_WIDTH_SQUARE_SIZE = 5
FIGURE_HEIGHT_SQUARE_SIZE = 5
FIGURE_HEIGHT_SIZE = 6.69
FIGURE_HEIGHT_SMALL_SIZE = 4
FIGURE_HEIGHT_LARGE_SIZE = 7.5

FOREGROUND_COLOR = '#0072B2'  # blue
BACKGROUND_COLOR = '#A9A9A9'  # dark grey
BACKGROUND_ALPHA = 0.5
FOREGROUND_ALPHA = 0.2

BBOX_STYLE = {'boxstyle': 'square,pad=0.0', 'lw': 0, 'fc': 'w', 'alpha': 0.8}
LABEL_STYLE = {'fontname': 'Roboto', 'weight': 'normal'}


def save_figure(filename_no_ext, rotate=None):
    filename = filename_no_ext + '.' + FORMAT

    print('Saving file', os.path.abspath(filename))
    # TODO: increase dpi for publication
    matplotlib.pyplot.savefig(filename, bbox_inches='tight', format=FORMAT, transparent=True, dpi=300)

    if rotate:
        image = PIL.Image.open(filename)
        rotated_image = image.rotate(rotate, expand=True)
        rotated_image.save(filename)


def plot_switch_performance(args):
    input_files_pattern = getattr(args, 'input-files-pattern')
    output_file = getattr(args, 'output', 'output')

    solution_bundles = [load_solution_bundle(file_system_item)
                        for file_system_item in glob.glob(input_files_pattern, recursive=False)]

    total_key_rate_ub = max(
        solution_bundle.metadata.total_keys_transferred.upper_bound for solution_bundle in solution_bundles)

    data = []
    for bundle in solution_bundles:
        for solution in bundle.solutions:
            data.append([bundle.metadata.switch_duration, solution.total_keys_transferred])
    data_frame = pandas.DataFrame(columns=['SwitchDuration', 'TotalKeyRate'], data=data)

    figure, axis = matplotlib.pyplot.subplots()
    points_handle = axis.scatter(data_frame.SwitchDuration, data_frame.TotalKeyRate, s=2)
    axis.set_ylim(bottom=0, top=1.1 * total_key_rate_ub)
    axis.grid(which='major', axis='y', linestyle='--')
    axis.set_xticks([0, 15, 30, 45, 60])
    axis.set_xlabel('Switch Duration')
    axis.set_ylabel('Total Keys')
    ub_handle = axis.axhline(y=total_key_rate_ub, linestyle='--')
    figure.tight_layout()
    axis.legend([points_handle, ub_handle], ['Example Solution', 'Upper Bound'],
                ncol=2,
                loc='upper center',
                bbox_to_anchor=(0.5, -0.12))
    figure.subplots_adjust(bottom=0.18)
    save_figure(output_file)

    matplotlib.pyplot.autoscale()
    save_figure(output_file + '_auto_scale')


def plot_bundle(args):
    input_file = getattr(args, 'input')
    output_file = getattr(args, 'output')
    raw_max_jobs = getattr(args, 'max_jobs')

    bundles = [load_solution_bundle(file_item) for file_item in glob.glob(input_file)]

    data = []
    for bundle in bundles:
        for solution in bundle.solutions:
            used_jobs_count = len([job for job in solution.jobs if job.keys_transferred > 0])
            data.append([used_jobs_count, solution.total_keys_transferred])

    if raw_max_jobs:
        max_jobs = int(raw_max_jobs)
    else:
        max_jobs = max(map(operator.itemgetter(0), data))
    min_jobs = min(map(operator.itemgetter(0), data))

    total_key_rate_ub = max(bundle.metadata.total_keys_transferred.upper_bound for bundle in bundles)

    data_frame = pandas.DataFrame(columns=['Jobs', 'TotalKeyRate'], data=data)

    figure, axis = matplotlib.pyplot.subplots()

    points_handle = axis.scatter(data_frame.Jobs, data_frame.TotalKeyRate, s=2)

    ub_handle = axis.axhline(y=total_key_rate_ub, linestyle='--')

    axis.set_ylim(bottom=0, top=1.1 * total_key_rate_ub)
    axis.grid(which='major', axis='y', linestyle='--')
    axis.set_xlim(left=min_jobs - 1, right=max_jobs + 1)
    axis.set_xticks(numpy.arange(min_jobs, max_jobs + 1, 1))
    axis.xaxis.set_major_formatter(matplotlib.ticker.StrMethodFormatter("{x}"))
    axis.set_xlabel('Data Transfers')
    axis.set_ylabel('Total Keys')
    axis.legend([points_handle, ub_handle], ['Example Solutions', 'Upper Bound'],
                ncol=2,
                loc='upper center',
                bbox_to_anchor=(0.5, -0.12))

    figure.tight_layout()
    figure.subplots_adjust(bottom=0.18)

    save_figure(output_file)
    matplotlib.pyplot.autoscale()
    save_figure(output_file + '_auto_scale')


def get_cities(names):
    cities = [quake.city.City.from_name(name) for name in names]
    cities.sort(key=operator.attrgetter('latitude'), reverse=True)
    return cities


def export_solution(args):
    output_file_no_ext = getattr(args, 'output')
    forecast_input_file = getattr(args, 'forecast_input')
    solution_input_file = getattr(args, 'solution_input')

    solution_bundle = load_solution_bundle(solution_input_file)
    cloud_cover = quake.cloud_cover.CloudCoverIndex.from_csv(forecast_input_file)

    columns = ['start_date', 'station', 'transfer_start', 'transfer_duration', 'transferred_keys', 'cloud_cover']
    records = []

    assert len(solution_bundle.solutions) == 1
    solution = solution_bundle.solutions[0]

    for job in solution.jobs:
        records.append([job.start_time.date(),
                        job.station,
                        job.start_time.time(),
                        str(job.end_time - job.start_time),
                        float(job.key_rate) / 100.0,
                        cloud_cover(job.station, job.start_time)])
    data_frame = pandas.DataFrame(columns=columns, data=records)
    with pandas.ExcelWriter(output_file_no_ext + '.xlsx') as excel_writer:
        data_frame.to_excel(excel_writer)
        excel_writer.save()


def plot_long_term_performance(args):
    data_dir = getattr(args, 'data_dir')
    solution_dir = getattr(args, 'solution_dir')

    simulation = quake.multistage_simulation.MultistageSimulation.load_from_dir(data_dir, solution_dir)
    frame = simulation.to_frame()

    cities = get_cities(map(operator.attrgetter('name'), simulation.stations))
    last_city_index = len(cities) - 1
    fig, axis = matplotlib.pyplot.subplots(len(cities), 1,
                                           sharex=True, figsize=(FIGURE_WIDTH_SIZE, FIGURE_HEIGHT_LARGE_SIZE))
    for index, city in enumerate(cities):
        station_frame = frame[frame[quake.multistage_simulation.STATION_KEY] == city.name]
        axis[index].plot(station_frame['Week'], station_frame['KeyTransferred'], color=FOREGROUND_COLOR)
        axis[index].annotate(city.name,
                             xy=(1.0, 0.80),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             bbox=BBOX_STYLE)
        if index != last_city_index:
            axis[index].get_xaxis().set_visible(False)
        # axis[index].set_ylim(0, 15000)
    axis[len(cities) // 2].set_ylabel('Keys Received')
    axis[-1].set_xlabel('Date')
    fig.tight_layout()
    fig.subplots_adjust(hspace=0.08)
    save_figure('long_term_performance_' + os.path.basename(solution_dir))

    fig, axis = matplotlib.pyplot.subplots(1, 1)
    for index, city in enumerate(cities):
        station_frame = frame[frame[quake.multistage_simulation.STATION_KEY] == city.name]
        axis.plot(station_frame['Week'], station_frame['KeyTransferred'], label=city.name)
    axis.set_xlabel('Date')
    axis.set_ylabel('Keys Received')
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.30)
    fig.legend(loc='lower center', ncol=3, bbox_to_anchor=(0.5, 0.0))
    save_figure('long_term_performance_v2_' + os.path.basename(solution_dir))

    min_lambda_series = frame.groupby(['Week'])['KeyRatio'].min()
    fig, axis = matplotlib.pyplot.subplots(1, 1)
    axis.plot(min_lambda_series)
    axis.set_xlabel('Date')
    axis.set_ylabel('Traffic Index')
    fig.tight_layout()
    save_figure('long_term_lambda_' + os.path.basename(solution_dir))


def load_transfer_index():
    return quake.transfer_rate.TransferRateIndex.from_odf('/home/pmateusz/dev/quake/network_share/key_rate/data_24_05eff.ods')


def load_communication_windows(data_dir: pathlib.Path, station: quake.city.City) -> typing.List[quake.weather.time_period.TimePeriod]:
    problems = []
    for file_name in os.listdir(str(data_dir)):
        if file_name.startswith('week_'):
            problem_path = os.path.join(str(data_dir), file_name)
            problems.append(quake.weather.problem.Problem.read_json(problem_path))
    problems.sort(key=lambda problem: problem.observation_period.begin)

    communication_windows = []
    for problem in problems:
        communication_windows.extend(problem.get_communication_windows(station))
    return communication_windows


def load_sunset_sunrise_index():
    root_dir = '/home/pmateusz/dev/quake/data/sunset_sunrise'
    file_paths = [os.path.join(root_dir, file_item) for file_item in os.listdir(root_dir)]
    return quake.sunset_sunrise.SunsetSunriseIndex.from_files(file_paths)


def plot_aggregate(args):
    data_directory = getattr(args, 'data_dir')

    elevation_data_frame_path = os.path.join(data_directory, 'filtered_elevation_data_frame.hdf')
    cloud_cover_data_frame_path = os.path.join(data_directory, 'cloud_cover_data_frame.hdf')
    transfer_frame_path = os.path.join(data_directory, 'transfer_data_frame.hdf')
    weather_corrected_frame_path = os.path.join(data_directory, 'weather_corrected_data_frame.hdf')

    sunset_sunrise_index = load_sunset_sunrise_index()

    if not os.path.isfile(elevation_data_frame_path):
        def get_hdf_file(directory, date):
            return os.path.join(directory, 'elevation_frame_' + date.strftime('%Y-%m-%d') + '.hdf')

        def reload_hdf_files(directory):
            filename_match = re.compile(r'^elevation_(?P<date>\d+-\d+-\d+).csv$')
            elevation_files = []
            for file_name in os.listdir(directory):
                match = filename_match.match(file_name)
                if not match:
                    continue

                date_time = datetime.datetime.strptime(match.group('date'), '%Y-%m-%d')
                file_path = os.path.join(directory, file_name)
                elevation_files.append((date_time, file_path))
                elevation_files.sort(key=operator.itemgetter(0))

            with warnings.catch_warnings():
                warnings.filterwarnings('ignore', category=tqdm.TqdmSynchronisationWarning)
                for date, file_path in tqdm.tqdm(elevation_files):
                    hdf_file_path = get_hdf_file(directory, date)
                    if not os.path.isfile(hdf_file_path):
                        data_frame = load_elevation(file_path)
                        data_frame.to_hdf(hdf_file_path, key='df', mode='w')

            ordered_dates = [date for date, file_path in elevation_files]
            return ordered_dates

        file_dates = reload_hdf_files(data_directory)
        filtered_data_frames = []
        for date in tqdm.tqdm(file_dates):
            data_frame = pandas.read_hdf(get_hdf_file(data_directory, date))
            cities = [quake.city.City.from_name(name) for name in data_frame.columns.values[1:]]

            def is_night(date_time):
                return any(sunset_sunrise_index.is_night(city, date_time) for city in cities)

            result_frame = data_frame[data_frame.max(axis=1) >= quake.transfer_rate.MIN_ANGLE]
            filtered_data_frames.append(result_frame[result_frame['Time'].apply(is_night)].copy())
        elevation_data_frame = pandas.concat(filtered_data_frames)
        elevation_data_frame.drop_duplicates('Time', inplace=True)
        elevation_data_frame.set_index('Time', inplace=True)
        elevation_data_frame.to_hdf(elevation_data_frame_path, key='df', mode='w')
    else:
        elevation_data_frame = pandas.read_hdf(elevation_data_frame_path)

    cities = get_cities(elevation_data_frame.columns.values)

    if not os.path.isfile(cloud_cover_data_frame_path):
        def load_cloud_cover(data_dir):
            filename_match = re.compile(r'^cloud_cover_(?P<date>\d+-\d+-\d+).csv$')

            cloud_cover_files = []
            for file_name in os.listdir(data_dir):
                match = filename_match.match(file_name)
                if not match:
                    continue

                date_time = datetime.datetime.strptime(match.group('date'), '%Y-%m-%d')
                file_path = os.path.join(data_dir, file_name)
                cloud_cover_files.append((date_time, file_path))
                cloud_cover_files.sort(key=operator.itemgetter(0))

            cloud_covers = []
            for date, file_path in cloud_cover_files:
                cloud_covers.append(quake.cloud_cover.CloudCoverIndex.from_csv(file_path))
            return quake.cloud_cover.CloudCoverIndex.merge(cloud_covers)

        cloud_cover_index = load_cloud_cover(data_directory)
        cloud_cover_rows = []
        for timestamp in elevation_data_frame.index.tolist():
            date_time = timestamp.to_pydatetime()

            row = []
            for city in cities:
                row.append(cloud_cover_index(city, date_time))

            cloud_cover_rows.append(row)
        cloud_cover_data_frame = pandas.DataFrame(columns=elevation_data_frame.columns.values,
                                                  data=cloud_cover_rows,
                                                  index=elevation_data_frame.index.values)
        cloud_cover_data_frame.to_hdf(cloud_cover_data_frame_path, key='df', mode='w')
    else:
        cloud_cover_data_frame = pandas.read_hdf(cloud_cover_data_frame_path)

    if not os.path.isfile(transfer_frame_path):
        transfer_index = load_transfer_index()

        transfer_rows = []
        for timestamp in tqdm.tqdm(elevation_data_frame.index.tolist(), desc='Building transfer data frame'):
            row = []
            for city in cities:
                elevation = elevation_data_frame.at[timestamp, city.name]
                row.append(transfer_index(elevation))

            transfer_rows.append(row)
        transfer_data_frame = pandas.DataFrame(columns=elevation_data_frame.columns.values,
                                               data=transfer_rows,
                                               index=elevation_data_frame.index.values)
        transfer_data_frame.to_hdf(transfer_frame_path, key='df', mode='w')
    else:
        transfer_data_frame = pandas.read_hdf(transfer_frame_path)

    if not os.path.isfile(weather_corrected_frame_path):
        weather_corrected_rows = []
        for timestamp in tqdm.tqdm(elevation_data_frame.index.tolist(), desc='Building weather corrected data frame'):
            row = []
            for city in cities:
                effective_transfer = (100.0 - cloud_cover_data_frame.at[timestamp, city.name]) / 100.0 \
                                     * transfer_data_frame.at[timestamp, city.name]
                assert effective_transfer >= 0.0
                row.append(effective_transfer)

            weather_corrected_rows.append(row)
        weather_corrected_transfer_frame = pandas.DataFrame(columns=elevation_data_frame.columns.values,
                                                            data=weather_corrected_rows,
                                                            index=elevation_data_frame.index.values)
        weather_corrected_transfer_frame.to_hdf(weather_corrected_frame_path, key='df', mode='w')
    else:
        weather_corrected_transfer_frame = pandas.read_hdf(weather_corrected_frame_path)

    agg_weather_corrected_transfer_frame = weather_corrected_transfer_frame.resample('D').sum()
    agg_cloud_cover_frame = cloud_cover_data_frame.resample('D').mean()

    agg_raw_observation_frames = []
    for city in cities:
        city_frame = elevation_data_frame[city.name].copy()
        city_frame_filtered = city_frame[city_frame >= quake.transfer_rate.MIN_ANGLE].copy()
        night_in_city = list(
            map(lambda x: sunset_sunrise_index.is_night(city, x.to_pydatetime()), city_frame_filtered.index.tolist()))
        city_frame_filtered = city_frame_filtered[night_in_city].copy()
        agg_raw_observation_frames.append(city_frame_filtered
                                          .resample('D')
                                          .count()
                                          .map(lambda v: pandas.Timedelta(seconds=v))
                                          .to_frame(city.name))

    agg_observation_frame = pandas.concat(agg_raw_observation_frames, axis=1)

    HSPACE_ADJUST = 0.25

    # plot observation time
    last_city_index = len(cities) - 1
    fig, axis = matplotlib.pyplot.subplots(len(cities), 1, sharex=True,
                                           figsize=(FIGURE_WIDTH_SIZE, FIGURE_HEIGHT_SIZE))
    for index, city in enumerate(cities):
        axis[index].plot(agg_observation_frame[city.name], color=FOREGROUND_COLOR)
        axis[index].yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(PandasTimeDeltaConverter()))
        axis[index].set_yticks([0, 15 * 60 * 10 ** 9, 30 * 60 * 10 ** 9])
        axis[index].set_ylim(0, 40 * 60 * 10 ** 9)
        axis[index].annotate(city.name,
                             xy=(1.0, 0.80),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             bbox=BBOX_STYLE)
        if index != last_city_index:
            axis[index].get_xaxis().set_visible(False)
    axis[len(cities) // 2].set_ylabel('Daily Observation Time [hh:mm]')
    axis[-1].set_xlabel('Date')
    fig.tight_layout()
    fig.subplots_adjust(hspace=HSPACE_ADJUST)
    save_figure('observation_time')

    # plot average cloud cover
    fig, axis = matplotlib.pyplot.subplots(len(cities), 1, sharex=True,
                                           figsize=(FIGURE_WIDTH_SIZE, FIGURE_HEIGHT_SIZE))
    for index, city in enumerate(cities):
        axis[index].plot(agg_cloud_cover_frame[city.name], color=FOREGROUND_COLOR)
        axis[index].set_yticks([0, 50, 100])
        axis[index].annotate(city.name,
                             xy=(1.0, 0.80),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             bbox=BBOX_STYLE)
        if index != last_city_index:
            axis[index].get_xaxis().set_visible(False)
    axis[len(cities) // 2].set_ylabel('Average Cloud Cover [%]')
    axis[-1].set_xlabel('Date')
    fig.tight_layout()
    fig.subplots_adjust(hspace=HSPACE_ADJUST)
    save_figure('average_cloud_cover')

    # plot cloud cover
    cloud_cover_correlation_frame = cloud_cover_data_frame.corr()
    fig, axis = matplotlib.pyplot.subplots()
    axis.imshow(cloud_cover_correlation_frame, cmap='cividis')

    for row, city_row in enumerate(cities):
        for column, city_column in enumerate(cities):
            axis.text(row, column, round(cloud_cover_correlation_frame.at[city_row.name, city_column.name], 2),
                      ha="center", va="center", color="w")
    axis.set_xticks(numpy.arange(len(cities)))
    axis.set_yticks(numpy.arange(len(cities)))
    axis.set_xticklabels(cities)
    axis.set_yticklabels(cities)
    axis.tick_params(top=False, bottom=True, labeltop=False, labelbottom=True)
    matplotlib.pyplot.setp(axis.get_xticklabels(), rotation=90, ha="right", rotation_mode="anchor")

    axis.set_xticks(numpy.arange(len(cities) + 1) - 0.5, minor=True)
    axis.set_yticks(numpy.arange(len(cities) + 1) - 0.5, minor=True)
    axis.tick_params(which='minor', bottom=False, left=False)
    fig.tight_layout()

    save_figure('cloud_cover_correlation')

    # plot max keys transferred
    fig, axis = matplotlib.pyplot.subplots(len(cities), 1, sharex=True,
                                           figsize=(FIGURE_WIDTH_SIZE, FIGURE_HEIGHT_SIZE))
    for index, city in enumerate(cities):
        axis[index].plot(agg_weather_corrected_transfer_frame[city.name], color=FOREGROUND_COLOR)
        axis[index].set_yticks([0, 5000, 10000, 15000])
        axis[index].annotate(city.name,
                             xy=(1.0, 0.80),
                             xycoords='axes fraction',
                             horizontalalignment='right',
                             verticalalignment='center',
                             bbox=BBOX_STYLE)
        if index != last_city_index:
            axis[index].get_xaxis().set_visible(False)
    axis[len(cities) // 2].set_ylabel('Maximum Keys Received')
    axis[-1].set_xlabel('Date')
    fig.tight_layout()
    fig.subplots_adjust(hspace=HSPACE_ADJUST)
    save_figure('maximum_keys_received')

    max_key_correlation_frame = agg_weather_corrected_transfer_frame.corr()
    fig, axis = matplotlib.pyplot.subplots()
    axis.imshow(max_key_correlation_frame, cmap='cividis', aspect='equal')

    for row, city_row in enumerate(cities):
        for column, city_column in enumerate(cities):
            axis.text(row, column, round(max_key_correlation_frame.at[city_row.name, city_column.name], 2),
                      ha="center", va="center", color="w")
    axis.set_xticks(numpy.arange(len(cities)))
    axis.set_yticks(numpy.arange(len(cities)))
    axis.set_xticklabels(cities)
    axis.set_yticklabels(cities)
    axis.tick_params(top=False, bottom=True, labeltop=False, labelbottom=True)
    matplotlib.pyplot.setp(axis.get_xticklabels(), rotation=90, ha="right", rotation_mode="anchor")

    axis.set_xticks(numpy.arange(len(cities) + 1) - 0.5, minor=True)
    axis.set_yticks(numpy.arange(len(cities) + 1) - 0.5, minor=True)
    axis.tick_params(which='minor', bottom=False, left=False)
    fig.tight_layout()

    save_figure('maximum_keys_correlation')

    # Transform to total seconds
    cumulative_index = pandas.date_range(start=agg_observation_frame.index.min(),
                                         end=agg_observation_frame.index.max()).values
    cumulative_min = agg_observation_frame.transpose().min().values / numpy.timedelta64(1, 's')
    cumulative_max = agg_observation_frame.transpose().max().values / numpy.timedelta64(1, 's')

    for city in cities:
        fig, axis = matplotlib.pyplot.subplots(1, 1, figsize=(FIGURE_WIDTH_SIZE, FIGURE_HEIGHT_SMALL_SIZE))

        # for other_city in cities:
        #     axis.plot(agg_observation_frame[other_city.name], c=gray_color)
        axis.fill_between(cumulative_index, cumulative_min, cumulative_max,
                          facecolor=FOREGROUND_COLOR, alpha=FOREGROUND_ALPHA)
        cumulative_city = agg_observation_frame[city.name].values / numpy.timedelta64(1, 's')

        axis.plot(cumulative_index, cumulative_city, c=FOREGROUND_COLOR)
        axis.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(NumpyTimeDeltaConverter()))
        axis.set_yticks([value * 60 for value in range(5, 40, 5)])
        axis.set_ylabel('Daily Observation Time [hh:mm]')
        axis.set_xlabel('Date')
        fig.tight_layout()
        save_figure('cumulative_observation_time_' + city.name)

    matplotlib.pyplot.close("all")


def plot_bottleneck(args):
    run_pattern = re.compile(r'^run_(?P<number>\d+)(:?_disturbed10)?$')

    data_dir = getattr(args, 'data_dir')
    solution_dirs = []

    min_run_number = int(getattr(args, 'min_run'))

    for file_item in os.listdir(data_dir):
        file_path = os.path.join(data_dir, file_item)
        if os.path.isdir(file_path):
            match = run_pattern.match(file_item)
            if match:
                run_number = int(match.group('number'))
                if run_number >= min_run_number:
                    solution_dirs.append((run_number, file_path))
    solution_dirs.sort(key=operator.itemgetter(0))

    city_rank = collections.defaultdict(list)
    for run_number, solution_dir in solution_dirs:
        simulation = quake.multistage_simulation.MultistageSimulation.load_from_dir(data_dir, solution_dir)
        bottleneck_counter = collections.Counter()
        bottlenecks = simulation.compute_bottlenecks()
        cities = set()
        for row_dict in bottlenecks:
            for city, rank in row_dict.items():
                cities.add(city)
                if rank == 0:
                    bottleneck_counter[city] += 1
        for city in cities:
            city_rank[city].append(bottleneck_counter[city] if city in bottleneck_counter else 0)

    for city in city_rank:
        samples = city_rank[city]
        median = statistics.median(samples)
        medians = [median] * len(samples)

        print(city, median, samples)
        # else:
        #     print(city, median, samples, scipy.stats.wilcoxon(samples, medians))


def plot_network_traffic(args):
    data_dir = getattr(args, 'data_dir')
    solution_dir = getattr(args, 'solution_dir')
    simulation = quake.multistage_simulation.MultistageSimulation.load_from_dir(data_dir, solution_dir)
    cities = get_cities(map(operator.attrgetter('name'), simulation.stations))
    data_frame = simulation.to_frame()

    data = []
    for city in cities:
        city_frame = data_frame[data_frame['Station'] == city.name].copy()
        min_week = city_frame['Week'].min()
        max_week = city_frame['Week'].max()
        filtered_city_frame = city_frame[(city_frame['Week'] < max_week) & (city_frame['Week'] > min_week)]

        data.append(filtered_city_frame['KeyTransferred'])

    fig, ax = matplotlib.pyplot.subplots(figsize=(FIGURE_WIDTH_SQUARE_SIZE, FIGURE_HEIGHT_SQUARE_SIZE))
    boxplot = ax.boxplot(data, flierprops=dict(marker='.'), medianprops=dict(color=FOREGROUND_COLOR))
    ax.set_xlabel('City')
    ax.set_ylabel('Weekly Keys Received')
    ax.set_xticklabels([city.name for city in cities], rotation=90)
    ax.set_ylim(bottom=0)
    fig.tight_layout()

    save_figure('network_traffic')


def compute_local_service_frame(frame, city):
    city_data_frame = frame[frame['Station'] == city.name].copy()
    city_data_frame.set_index('Week', inplace=True)
    city_data_frame.sort_index(inplace=True)

    failure_dates = dict()
    cumulative_key_buffer = 0
    data = []
    for index in city_data_frame.index:
        key_transferred = city_data_frame.at[index, 'KeyTransferred']
        cumulative_key_buffer += key_transferred
        data.append((index, cumulative_key_buffer))

    consumption_failures = []
    current_step_consumption = 0
    current_failures = 0
    while current_failures < len(data):
        cumulative_consumption = 0
        current_failures = 0
        for date, buffer_size in data:
            cumulative_consumption += current_step_consumption
            if cumulative_consumption > buffer_size:
                current_failures += 1
                if date not in failure_dates:
                    failure_dates[date] = current_step_consumption
        consumption_failures.append((current_step_consumption, current_failures))
        current_step_consumption += 1

    service_data_frame = pandas.DataFrame(columns=['Consumption', 'Failures'], data=consumption_failures)
    days_length_vector = numpy.full(len(service_data_frame), float(len(data)))
    service_data_frame['SuccessRate'] \
        = (days_length_vector - service_data_frame['Failures']) / days_length_vector
    return service_data_frame, failure_dates


def compute_global_service_frame(city_date_rate_failure, simulation):
    dates = set()
    for city in city_date_rate_failure:
        for date in city_date_rate_failure[city]:
            dates.add(date)
    dates = list(dates)
    dates.sort()

    lambda_growth_rate = 0
    data = []
    dates_failed = set()
    while len(dates_failed) < len(dates):
        dates_failed = set()
        for city in city_date_rate_failure:
            for date in dates:
                if city_date_rate_failure[city][date] <= lambda_growth_rate * simulation.transfer_rates[city]:
                    dates_failed.add(date)
        data.append(
            (lambda_growth_rate, len(dates_failed), (float(len(dates)) - float(len(dates_failed))) / len(dates)))
        lambda_growth_rate += 10
    return pandas.DataFrame(columns=['Consumption', 'Failures', 'SuccessRate'], data=data)


def plot_service_level(args):
    data_dir = getattr(args, 'data_dir')
    solution_dir = getattr(args, 'solution_dir')
    simulation = quake.multistage_simulation.MultistageSimulation.load_from_dir(data_dir, solution_dir)

    match = re.match('[^\d\s]+_(?P<number>\d+)', solution_dir)
    run_number = int(match.group('number'))
    data_frame = simulation.to_frame()
    min_week = data_frame['Week'].min()
    filtered_frame = data_frame[(data_frame['Week'] > min_week)]

    def __plot_service_level(data_frame):
        frame_to_use = data_frame.copy()

        frame_to_use['Difference99'] = (frame_to_use['SuccessRate'] - 0.99).abs()
        best_99point_difference = frame_to_use['Difference99'].min()
        max_consumption_at_99service_level = frame_to_use[
            frame_to_use['Difference99'] == best_99point_difference].sort_values('Consumption', ascending=False)

        point_99success_rate = max_consumption_at_99service_level['SuccessRate'].iloc[0]
        point_99consumption = max_consumption_at_99service_level['Consumption'].iloc[0]

        frame_to_use['Difference100'] = (frame_to_use['SuccessRate'] - 1.0).abs()
        best_100point_difference = frame_to_use['Difference100'].min()
        max_consumption_at_100service_level = frame_to_use[
            frame_to_use['Difference100'] == best_100point_difference].sort_values('Consumption', ascending=False)

        point_100success_rate = max_consumption_at_100service_level['SuccessRate'].iloc[0]
        point_100consumption = max_consumption_at_100service_level['Consumption'].iloc[0]

        fig, axis = matplotlib.pyplot.subplots(figsize=(FIGURE_WIDTH_SQUARE_SIZE, FIGURE_HEIGHT_SQUARE_SIZE))
        axis.plot(frame_to_use['Consumption'], frame_to_use['SuccessRate'])

        if point_99consumption != point_100consumption:
            axis.plot([point_99consumption], [point_99success_rate], '.', c=FOREGROUND_COLOR)
            axis.annotate('({0}, {1:.3f})'.format(point_99consumption, point_99success_rate),
                          xy=(point_99consumption, point_99success_rate), textcoords='figure fraction',
                          xytext=(0.4, 0.75),
                          arrowprops=dict(facecolor='black', arrowstyle="->", connectionstyle="arc3"))

            axis.plot([point_100consumption], [point_100success_rate], '.', c=FOREGROUND_COLOR)
            axis.annotate('({0}, {1:.3f})'.format(point_100consumption, point_100success_rate),
                          xy=(point_100consumption, point_100success_rate), textcoords='figure fraction',
                          xytext=(0.2, 0.8),
                          arrowprops=dict(facecolor='black', arrowstyle="->", connectionstyle="arc3"))
        else:
            axis.plot([point_100consumption], [point_100success_rate], '.', c=FOREGROUND_COLOR)
            axis.annotate('({0}, {1:.3f})'.format(point_100consumption, point_100success_rate),
                          xy=(point_100consumption, point_100success_rate), textcoords='figure fraction',
                          xytext=(0.4, 0.75),
                          arrowprops=dict(facecolor='black', arrowstyle="->", connectionstyle="arc3"))

        return fig, axis

    city_date_keyrate_failure = dict()
    for city in simulation.stations:
        service_data_frame, city_date_failures = compute_local_service_frame(filtered_frame, city)
        city_date_keyrate_failure[city] = city_date_failures
        fig, axis = __plot_service_level(service_data_frame)
        axis.set_xlabel('Weekly Key Consumption')
        axis.set_ylabel('Service Level')
        axis.set_xlim(left=0)
        axis.set_ylim(bottom=0)
        fig.tight_layout()
        save_figure('service_level_' + city.name + '_run' + str(run_number))
        matplotlib.pyplot.close(fig)

    service_level_frame = compute_global_service_frame(city_date_keyrate_failure, simulation)
    fig, axis = __plot_service_level(service_level_frame)
    axis.set_xlabel('Traffic Index')
    axis.set_ylabel('Global Service Level')
    axis.set_xlim(left=0)
    axis.set_ylim(bottom=0)
    fig.tight_layout()
    save_figure('service_level_global_run' + str(run_number))
    matplotlib.pyplot.close(fig)


def plot_all_service_levels(args):
    config_static = [('/home/pmateusz/dev/quake/data/simulation_upgrade',
                      '/home/pmateusz/dev/quake/data/simulation_upgrade/run_1')]
    config_perturbed = [('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_1',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_1_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_2',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_2_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_3',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_3_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_4',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_4_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_5',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_5_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_6',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_6_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_7',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_7_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_8',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_8_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_9',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_9_disturbed10'),
                        ('/home/pmateusz/dev/quake/network_share/simulation_upgrade_disturbed_10',
                         '/home/pmateusz/dev/quake/network_share/simulation_upgrade/run_10_disturbed10')]

    # config_static = [('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_11'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_12'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_13'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_14'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_15'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_16'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_17'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_18'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_19'),
    #                  ('/home/pmateusz/dev/quake/data/simulation', '/home/pmateusz/dev/quake/data/simulation/run_20')]

    # config_perturbed = [('/home/pmateusz/dev/quake/data/simulation_disturbed_1',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_21_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_2',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_22_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_3',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_23_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_4',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_24_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_5',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_25_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_6',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_26_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_7',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_27_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_8',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_28_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_9',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_29_disturbed'),
    #                     ('/home/pmateusz/dev/quake/data/simulation_disturbed_10',
    #                      '/home/pmateusz/dev/quake/data/simulation/run_30_disturbed')]

    # config_small_perturbed = [('/home/pmateusz/dev/quake/data/simulation_disturbed_11',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_31_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_12',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_32_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_13',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_33_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_14',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_34_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_15',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_35_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_16',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_36_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_17',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_37_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_18',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_38_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_19',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_39_disturbed'),
    #                           ('/home/pmateusz/dev/quake/data/simulation_disturbed_20',
    #                            '/home/pmateusz/dev/quake/data/simulation/run_40_disturbed')]

    def __plot(config, output_file):
        service_level_frames = []
        for data_dir, solution_file in config:
            simulation = quake.multistage_simulation.MultistageSimulation.load_from_dir(os.path.abspath(data_dir),
                                                                                        os.path.abspath(solution_file))
            city_date_rate_failures = dict()
            for city in simulation.stations:
                data_frame = simulation.to_frame()
                min_week = data_frame['Week'].min()
                filtered_frame = data_frame[(data_frame['Week'] > min_week)]
                _, city_date_failure = compute_local_service_frame(filtered_frame, city)
                city_date_rate_failures[city] = city_date_failure
            service_level_frame = compute_global_service_frame(city_date_rate_failures, simulation)
            service_level_frames.append(service_level_frame)
            london_transfer_rate = simulation.transfer_rates[quake.city.LONDON]
            consumption_100 = service_level_frame[service_level_frame['Failures'] == 0]['Consumption'].max()
            consumption_98 = service_level_frame[service_level_frame['Failures'] == 1]['Consumption'].max()
            print(london_transfer_rate, london_transfer_rate * consumption_100, london_transfer_rate * consumption_98)

        fig, axis = matplotlib.pyplot.subplots(figsize=(FIGURE_WIDTH_SQUARE_SIZE, FIGURE_HEIGHT_SQUARE_SIZE))
        for frame in service_level_frames:
            axis.plot(frame['Consumption'], frame['SuccessRate'], c=FOREGROUND_COLOR)
        axis.set_xlabel('Traffic Index')
        axis.set_ylabel('Global Service Level')
        axis.set_xlim(left=0)
        axis.set_ylim(bottom=0)
        fig.tight_layout()
        save_figure(output_file)
        matplotlib.pyplot.close(fig)

    # __plot(config_static, 'service_level_global_run')
    __plot(config_perturbed, 'service_level_global_run_perturbed10')
    # __plot(config_small_perturbed, 'service_level_global_run_small_perturbed')


def plot_week_performance(args):
    problem_file = getattr(args, 'problem-file')
    solution_file = getattr(args, 'solution-file')

    import quake.weather.problem
    import quake.weather.solution

    problem = quake.weather.problem.Problem.read_json(problem_file)
    solution = quake.weather.solution.Solution.read_json(solution_file)

    key_rate_frame = problem.get_key_rate_frame()

    split_points = []
    times = key_rate_frame.index.tolist()
    for position in range(1, len(times)):
        time_delta = times[position] - times[position - 1]
        assert time_delta.seconds >= 1
        if time_delta.seconds > 1:
            split_points.append(times[position])

    assert len(split_points) > 1

    cities = problem.stations

    split_frames = []
    split_frames.append(key_rate_frame[:split_points[0] - pandas.Timedelta(seconds=1)])
    for split_point in range(1, len(split_points)):
        split_frames.append(key_rate_frame[
                            split_points[split_point - 1]:split_points[split_point] - pandas.Timedelta(seconds=1)])
    split_frames.append(key_rate_frame[split_points[-1]:])

    split_jobs = [list() for _ in split_frames]
    for observation in solution.observations:
        for index, frame in enumerate(split_frames):
            if observation.period.end < frame.index.min():
                break
            if observation.period.begin > frame.index.max():
                continue
            if (frame.index.min() <= observation.period.begin < frame.index.max()) \
                    or (observation.period.begin < frame.index.min() < observation.period.end):
                split_jobs[index].append(observation)

    frame_ratios = [(frame.index.max() - frame.index.min()).seconds for frame in split_frames]

    fig = matplotlib.pyplot.figure(figsize=(FIGURE_HEIGHT_LARGE_SIZE, FIGURE_WIDTH_SIZE))
    grid_spec = matplotlib.gridspec.GridSpec(len(cities), len(frame_ratios), width_ratios=frame_ratios)

    def date_formatter(x, pos=None):
        try:
            local_datetime = matplotlib.dates.num2date(x)
            return local_datetime.strftime('%d-%m %H:%M')
        except ValueError:
            return None

    for city_index, city in enumerate(cities):
        first_row_ax = fig.add_subplot(grid_spec[city_index * len(frame_ratios)])
        for frame_index in range(1, len(frame_ratios)):
            fig.add_subplot(grid_spec[city_index * len(frame_ratios) + frame_index], sharey=first_row_ax, sharex=first_row_ax)

    def generate_ticks(start, end):
        time_step = pandas.Timedelta(minutes=5)
        ticks = []

        tick = pandas.Timestamp(year=start.year,
                                month=start.month,
                                day=start.day,
                                hour=start.hour,
                                minute=(start.minute // 5) * 5,
                                second=0)
        while tick < end:
            if tick >= start:
                ticks.append(tick)
            tick += time_step

        if not ticks:
            seconds_to_move = (end - start).seconds // 2
            middle_slot = start + pandas.Timedelta(seconds=seconds_to_move)
            ticks.append(pandas.Timestamp(year=middle_slot.year, month=middle_slot.month, day=middle_slot.day,
                                          hour=middle_slot.hour, minute=middle_slot.minute, second=1))
        return ticks

    def setup_y_labels(axis: matplotlib.pyplot.axis, hide=True):
        axis.set_ylim(bottom=0, top=40)
        if hide:
            axis.yaxis.set_ticklabels([])
            axis.get_yaxis().set_visible(False)
        else:
            axis.set_yticks([0, 25])

    def setup_x_labels(axis: matplotlib.pyplot.axis, frame: pandas.DataFrame, hide=True):
        if hide:
            axis.xaxis.set_ticklabels([])
            axis.get_xaxis().set_visible(False)
        else:
            matplotlib.pyplot.setp(axis.get_xticklabels(), rotation=90)
            axis.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(date_formatter))
            ticks = generate_ticks(frame.index.min(), frame.index.max())
            if len(ticks) >= 1:
                axis.xaxis.set_ticks(ticks, minor=False)
            else:
                minute_interval = (frame.index.max() - frame.index.min()).seconds // 60
                axis.xaxis.set_major_locator(matplotlib.dates.MinuteLocator(interval=minute_interval))

    def plot_transfer_rate(axis: matplotlib.pyplot.Axes, data):
        axis.plot(data, c=FOREGROUND_COLOR, linewidth=1.0, alpha=0.8)

    for city_index, city in enumerate(cities[:-1]):
        first_row_ax = matplotlib.pyplot.subplot(grid_spec[city_index * len(split_frames)])
        plot_transfer_rate(first_row_ax, key_rate_frame[city][split_frames[0].index.min():split_frames[0].index.max()])
        setup_y_labels(first_row_ax, hide=False)
        setup_x_labels(first_row_ax, split_frames[0], hide=True)
        for split_index, frame in enumerate(split_frames[1:], start=1):
            ax = matplotlib.pyplot.subplot(grid_spec[city_index * len(split_frames) + split_index])
            plot_transfer_rate(ax, key_rate_frame[city][frame.index.min():frame.index.max()])
            setup_x_labels(ax, frame, hide=True)
            setup_y_labels(ax)

    last_city = cities[-1]
    last_city_index = len(cities) - 1
    first_row_ax = matplotlib.pyplot.subplot(grid_spec[last_city_index * len(split_frames)])
    plot_transfer_rate(first_row_ax, key_rate_frame[last_city][split_frames[0].index.min():split_frames[0].index.max()])
    setup_x_labels(first_row_ax, split_frames[0], hide=False)
    setup_y_labels(first_row_ax, hide=False)

    for split_index, frame in enumerate(split_frames[1:], start=1):
        ax = matplotlib.pyplot.subplot(grid_spec[last_city_index * len(split_frames) + split_index])
        plot_transfer_rate(ax, key_rate_frame[last_city][frame.index.min():frame.index.max()])
        setup_x_labels(ax, frame, hide=False)
        setup_y_labels(ax)

    for city_index, city in enumerate(cities):
        last_ax = matplotlib.pyplot.subplot(grid_spec[city_index * len(split_frames) + len(split_frames) - 1])
        last_ax.annotate(city,
                         xy=(1.0, 0.80),
                         xycoords='axes fraction',
                         horizontalalignment='right',
                         verticalalignment='center',
                         bbox=BBOX_STYLE)

        for frame_index, frame in enumerate(split_frames):
            jobs = [job for job in split_jobs[frame_index] if job.station == city]
            if jobs:
                ax = matplotlib.pyplot.subplot(grid_spec[city_index * len(split_frames) + frame_index])
                for job in jobs:
                    series = frame[job.period.begin:job.period.end][job.station]
                    ax.plot(series, c='black', linewidth=2.0)

    label_x_index = (len(cities) - 1) * len(split_frames) + len(split_frames) // 2
    ax = matplotlib.pyplot.subplot(grid_spec[label_x_index])
    ax.set_xlabel('Time')

    label_y_index = (len(cities) // 2) * len(split_frames)
    ax = matplotlib.pyplot.subplot(grid_spec[label_y_index])
    ax.set_ylabel('Key Rate [1/s]')

    grid_spec.update(left=0.05, right=0.98, bottom=0.25, top=0.98, wspace=0.30, hspace=0.08)
    save_figure('solution_week_' + str(problem.observation_period.begin.date()), rotate=270)


def plot_communication_window(args):
    data_directory = getattr(args, 'data_dir')
    city = quake.city.from_name(getattr(args, 'station'))

    communication_windows = load_communication_windows(data_directory, city)
    sunset_sunrise_index = load_sunset_sunrise_index()

    date_index = list(set(map(lambda period: period.begin.date(), communication_windows)))
    date_index.sort()

    min_date_time = min(period.begin for period in communication_windows)
    max_date_time = max(period.end for period in communication_windows)
    ref_start_date = min_date_time.date()
    ref_end_date = (min_date_time + datetime.timedelta(days=1)).date()
    ref_start_date_time = datetime.datetime.combine(ref_start_date, datetime.time(hour=12, minute=0))
    ref_end_date_time = datetime.datetime.combine(ref_end_date, datetime.time(hour=12, minute=0))

    sunset_sunrise_data = []
    for date in date_index:
        sunrise_time = sunset_sunrise_index.sunrise(city, date)
        sunset_time = sunset_sunrise_index.sunset(city, date)
        sunset_sunrise_data.append([datetime.datetime.combine(ref_end_date, sunrise_time), datetime.datetime.combine(ref_start_date, sunset_time)])
    sunset_sunrise_frame = pandas.DataFrame(columns=['Sunrise', 'Sunset'], data=sunset_sunrise_data, index=date_index)

    figure, axis = matplotlib.pyplot.subplots(1, 1, figsize=(FIGURE_WIDTH_SIZE, FIGURE_HEIGHT_SQUARE_SIZE))
    axis.fill_between(date_index, sunset_sunrise_frame['Sunrise'].values, sunset_sunrise_frame['Sunset'].values,
                      interpolate=True,
                      facecolor=BACKGROUND_COLOR,
                      alpha=BACKGROUND_ALPHA)

    def map_datetime(value):
        if value.time() >= datetime.time(hour=12, minute=0):
            reference_date = ref_start_date
        else:
            reference_date = ref_end_date
        return datetime.datetime.combine(reference_date, value.time())

    for time_period in communication_windows:
        # the plot is showing night between two days, to maintain line vertical we overwrite date in the second day
        if time_period.begin.date() != time_period.end.date():
            axis.plot([time_period.begin.date(), time_period.begin.date()],
                      [map_datetime(time_period.begin), map_datetime(time_period.end)], '-', color=FOREGROUND_COLOR)
        else:
            axis.plot([time_period.begin.date(), time_period.end.date()],
                      [map_datetime(time_period.begin), map_datetime(time_period.end)], '-', color=FOREGROUND_COLOR)

    def date_time_formatter(x, pos=None):
        try:
            local_datetime = matplotlib.dates.num2date(x)
            return local_datetime.strftime('%H:%M')
        except ValueError:
            return None

    axis.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(date_time_formatter))
    axis.set_xlabel('Date')
    axis.set_ylabel('Time [hh:mm]')
    axis.set_ylim(bottom=ref_start_date_time, top=ref_end_date_time)
    axis.set_xlim(left=min_date_time, right=max_date_time)
    axis.set_xticks([datetime.date(min_date_time.year, 1, 1),
                     datetime.date(min_date_time.year, 4, 1),
                     datetime.date(min_date_time.year, 8, 1),
                     datetime.date(min_date_time.year, 12, 1)])
    figure.tight_layout()
    save_figure('observation_time_' + city.name)

    axis.set_xlim(left=datetime.date(min_date_time.year, 7, 1), right=datetime.date(min_date_time.year, 9, 15))
    axis.set_xticks([datetime.date(min_date_time.year, 7, 1), datetime.date(min_date_time.year, 8, 1), datetime.date(min_date_time.year, 9, 1)])
    axis.set_ylim(bottom=datetime.datetime.combine(ref_start_date, datetime.time(22, 30)),
                  top=datetime.datetime.combine(ref_end_date_time, datetime.time(00, 00)))
    figure.tight_layout()
    save_figure('observation_time_magnified_' + city.name)


def plot_weights_disturbed(args):
    data_dir = os.path.abspath(getattr(args, 'data_dir'))

    data_files = [('./simulation_disturbed_1/week_2018-01-01_fraction020.dzn', ['./simulation/run_21_disturbed']),
                  ('./simulation_disturbed_2/week_2018-01-01_fraction020.dzn', ['./simulation/run_22_disturbed']),
                  ('./simulation_disturbed_3/week_2018-01-01_fraction020.dzn', ['./simulation/run_23_disturbed']),
                  ('./simulation_disturbed_4/week_2018-01-01_fraction020.dzn', ['./simulation/run_24_disturbed']),
                  ('./simulation_disturbed_5/week_2018-01-01_fraction020.dzn', ['./simulation/run_25_disturbed']),
                  ('./simulation_disturbed_6/week_2018-01-01_fraction010.dzn', ['./simulation/run_26_disturbed']),
                  ('./simulation_disturbed_7/week_2018-01-01_fraction030.dzn', ['./simulation/run_27_disturbed']),
                  ('./simulation_disturbed_8/week_2018-01-01_fraction030.dzn', ['./simulation/run_28_disturbed']),
                  ('./simulation_disturbed_9/week_2018-01-01_fraction040.dzn', ['./simulation/run_29_disturbed']),
                  ('./simulation_disturbed_10/week_2018-01-01_fraction050.dzn', ['./simulation/run_30_disturbed'])]

    data_files_small_perturbation = [
        ('./simulation_disturbed_11/week_2018-01-01_fraction010.dzn', ['./simulation/run_31_disturbed']),
        ('./simulation_disturbed_12/week_2018-01-01_fraction010.dzn', ['./simulation/run_32_disturbed']),
        ('./simulation_disturbed_13/week_2018-01-01_fraction010.dzn', ['./simulation/run_33_disturbed']),
        ('./simulation_disturbed_14/week_2018-01-01_fraction010.dzn', ['./simulation/run_34_disturbed']),
        ('./simulation_disturbed_15/week_2018-01-01_fraction010.dzn', ['./simulation/run_35_disturbed']),
        ('./simulation_disturbed_16/week_2018-01-01_fraction010.dzn', ['./simulation/run_36_disturbed']),
        ('./simulation_disturbed_17/week_2018-01-01_fraction010.dzn', ['./simulation/run_37_disturbed']),
        ('./simulation_disturbed_18/week_2018-01-01_fraction010.dzn', ['./simulation/run_38_disturbed']),
        ('./simulation_disturbed_19/week_2018-01-01_fraction010.dzn', ['./simulation/run_39_disturbed']),
        ('./simulation_disturbed_20/week_2018-01-01_fraction010.dzn', ['./simulation/run_40_disturbed'])]

    upgraded_data_files_small_perturbation = [
        ('./simulation_upgrade_disturbed_1/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_1_disturbed10']),
        ('./simulation_upgrade_disturbed_2/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_2_disturbed10']),
        ('./simulation_upgrade_disturbed_3/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_3_disturbed10']),
        ('./simulation_upgrade_disturbed_4/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_4_disturbed10']),
        ('./simulation_upgrade_disturbed_5/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_5_disturbed10']),
        ('./simulation_upgrade_disturbed_6/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_6_disturbed10']),
        ('./simulation_upgrade_disturbed_7/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_7_disturbed10']),
        ('./simulation_upgrade_disturbed_8/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_8_disturbed10']),
        ('./simulation_upgrade_disturbed_9/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_9_disturbed10']),
        ('./simulation_upgrade_disturbed_10/week_2018-01-01_fraction010.dzn',
         ['./simulation_upgrade/run_10_disturbed10'])
    ]

    def __plot_weights_disturbed(data_files, prefix):
        configurations = []
        for problem_file, solution_files in data_files:
            problem_file_to_use = os.path.join(data_dir, problem_file)
            loader = quake.minizinc.MiniZincLoader()
            with open(problem_file_to_use, 'r') as file_stream:
                model = loader.load(file_stream)
            stations = [quake.city.City.from_name(name) for name in model['STATION'][1:]]
            transfer_shares = model['transfer_share'][1:]
            station_weights = {station: weight for station, weight in zip(stations, transfer_shares)}
            solutions = [
                quake.multistage_simulation.MultistageSimulation.load_from_dir(os.path.dirname(problem_file_to_use),
                                                                               os.path.join(data_dir, solution_file))
                for solution_file in solution_files]
            configurations.append((station_weights, solutions))

        stations = set()
        for station_weights, _ in configurations:
            for city in station_weights:
                stations.add(city)

        for city in stations:
            configurations_to_use = [(station_weights, solutions) for station_weights, solutions in configurations
                                     if city in station_weights]
            configurations_to_use.sort(key=lambda x: x[0][city])

            weight_data_records = []
            for station_weights, solutions in configurations_to_use:
                data = []
                for solution in solutions:
                    frame = solution.to_frame()
                    city_frame = frame[frame['Station'] == city.name].copy()
                    min_week = city_frame['Week'].min()
                    max_week = city_frame['Week'].max()
                    filtered_city_frame = city_frame[(city_frame['Week'] < max_week) & (city_frame['Week'] > min_week)]
                    data.extend(filtered_city_frame['KeyTransferred'].tolist())
                weight_data_records.append((station_weights[city], data))
            boxplot_data = [data for _, data in weight_data_records]

            fig, ax = matplotlib.pyplot.subplots(figsize=(FIGURE_WIDTH_SQUARE_SIZE, FIGURE_HEIGHT_SQUARE_SIZE))
            ax.boxplot(boxplot_data, flierprops=dict(marker='.'), medianprops=dict(color=FOREGROUND_COLOR))
            ax.set_xticklabels(["{0:.4f}".format(weight) for weight, _ in weight_data_records], rotation=90)

            ax.set_xlabel('Weight')
            ax.set_ylabel('Keys Received')
            fig.tight_layout()
            save_figure('noisy_weight_boxplot_{0}_{1}'.format(prefix, city.name))

    # __plot_weights_disturbed(data_files, 'varied')
    __plot_weights_disturbed(upgraded_data_files_small_perturbation, '10')


class ModelWrapper:

    def __init__(self, data_model, weather_model):
        self.__data_model = data_model
        self.__weather_model = weather_model

        start_time = datetime.datetime.strptime(self.__data_model['start_time'], '%Y-%b-%d %H:%M:%S')
        key_rate = self.__data_model['key_rate']
        time_offset = self.__data_model['time_offset']
        self.__stations = [quake.city.from_name(station_name) for station_name in self.__data_model['STATION']]

        data = []
        weather_data = []
        for offset_index, current_offset in enumerate(time_offset):
            current_time = start_time + datetime.timedelta(seconds=current_offset)
            data_row = [current_time]
            weather_data_row = [current_time]
            for station_index in range(len(self.__stations)):
                station = self.__stations[station_index]
                transfer_rate = key_rate[station_index][offset_index]
                cloud_cover = self.__weather_model(station, current_time)
                data_row.append(key_rate[station_index][offset_index])
                weather_data_row.append((100.0 - cloud_cover) * transfer_rate)
            data.append(data_row)
            weather_data.append(weather_data_row)

        columns = ['Time']
        columns.extend(self.__stations)

        self.__key_rate_frame = pandas.DataFrame(columns=columns, data=data)
        self.__key_rate_frame.set_index('Time', inplace=True)

        self.__weather_key_rate_frame = pandas.DataFrame(columns=columns, data=weather_data)
        self.__weather_key_rate_frame.set_index('Time', inplace=True)

        time_offset = self.__data_model['time_offset']
        step_duration = self.__data_model['step_duration']
        switch_duration = self.__data_model['switch_duration']

        reconfiguration_data = [[start_time, 0]]
        for time_index in range(1, len(time_offset)):
            current_time = start_time + datetime.timedelta(seconds=time_offset[time_index])
            current_switch_duration \
                = max(0, switch_duration - (time_offset[time_index] - time_offset[time_index - 1] - step_duration))
            reconfiguration_data.append([current_time, current_switch_duration])
        self.__reconfiguration_time_frame = pandas.DataFrame(columns=['Time', 'ReconfigurationTime'],
                                                             data=reconfiguration_data)
        self.__reconfiguration_time_frame.set_index('Time', inplace=True)

        assert len(self.__key_rate_frame) == len(self.__weather_key_rate_frame)

    def traffic_index(self, solution, station):
        keys_transferred = float(solution.keys_transferred(station))

        transfer_share = self.__data_model['transfer_share']
        station_index = self.__stations.index(station)
        station_transfer_share = transfer_share[station_index]
        if station_transfer_share == 0.0:
            return float('inf')

        return int(math.floor(keys_transferred / station_transfer_share))

    def keys_transferred(self, station, transfer_start, transfer_end):
        assert transfer_start <= transfer_end

        begin_index = pandas.Timestamp(transfer_start)
        end_index = ModelWrapper.__get_end_index(transfer_start, transfer_end)
        series = self.__key_rate_frame.loc[begin_index:end_index][station]
        # for index, value in series.iteritems():
        #     print(pandas.to_datetime(index).strftime('%Y-%b-%d %H:%M:%S'), value)
        return series.sum()

    def weather_adjusted_keys_transferred(self, station, transfer_start, transfer_end):
        assert transfer_start <= transfer_end

        begin_index = pandas.Timestamp(transfer_start)
        end_index = ModelWrapper.__get_end_index(transfer_start, transfer_end)
        series = self.__weather_key_rate_frame.loc[begin_index:end_index][station]
        # for index, value in series.iteritems():
        #
        #     value_to_use = value
        #     decimal_part = value - int(value)
        #     if decimal_part == 0.0:
        #         value_to_use = int(value)
        #     print(pandas.to_datetime(index).strftime('%Y-%b-%d %H:%M:%S'), value_to_use)
        return series.sum()

    def reconfiguration_time(self, start_time):
        index = pandas.Timestamp(start_time)
        reconfiguration_seconds = 0
        if index in self.__reconfiguration_time_frame.index:
            reconfiguration_seconds = self.__reconfiguration_time_frame.loc[index]['ReconfigurationTime'].item()
        return datetime.timedelta(seconds=reconfiguration_seconds)

    @property
    def stations(self):
        return self.__stations

    @staticmethod
    def __get_end_index(transfer_start, transfer_end):
        transfer_end_to_use = transfer_end
        if transfer_start < transfer_end:
            transfer_end_to_use -= datetime.timedelta(seconds=1)
        return pandas.Timestamp(transfer_end_to_use)


def plot_compare(args):
    problem_path = getattr(args, 'problem')
    weather_path = getattr(args, 'weather')
    baseline_solution_path = getattr(args, 'baseline_solution')
    other_solution_path = getattr(args, 'other_solution')

    data_loader = quake.minizinc.MiniZincLoader()
    with open(problem_path, 'r') as file_stream:
        data_model = data_loader.load(file_stream)

    weather_index = quake.cloud_cover.CloudCoverIndex.from_csv(weather_path)

    model = ModelWrapper(data_model, weather_index)

    baseline_solution = load_solution_bundle(baseline_solution_path).solutions[0]
    other_solution = load_solution_bundle(other_solution_path).solutions[0]

    regular_stations = [station for station in model.stations if station != quake.city.NONE]

    columns = ['Traffic Index']
    for station in regular_stations:
        columns.append(station.name)

    def get_solution_metrics(solution):
        traffic_indices = [model.traffic_index(solution, station) for station in regular_stations]
        traffic_index = min(traffic_indices)
        metrics = [traffic_index]
        for station in regular_stations:
            metrics.append(solution.keys_transferred(station))
        return metrics

    data = [get_solution_metrics(baseline_solution), get_solution_metrics(other_solution)]
    frame = pandas.DataFrame(data=data, columns=columns)
    print(tabulate.tabulate(frame, headers='keys', tablefmt='psql'))


def plot_key_rate(args):
    transfer_index = load_transfer_index()
    data_frame = transfer_index.data_frame

    bit_rate_frame = data_frame[data_frame['Config'] == '633'].copy()

    fig, ax = matplotlib.pyplot.subplots(1, 1)
    ax.plot(bit_rate_frame['Elevation'].values, bit_rate_frame['BitRate'].values)
    ax.set_ylim(bottom=0)
    ax.set_ylabel('Bitrate [kbit/s]')
    ax.set_xlabel('Elevation [°]')
    ax.set_xticks([0, 15, 30, 45, 60, 75, 90])
    fig.tight_layout()

    save_figure('key_rate')


def parse_args():
    parser = argparse.ArgumentParser(prog='quake-plot')
    subparsers = parser.add_subparsers(title='commands', dest='command')

    optics_performance_parser = subparsers.add_parser('switch-time-performance')
    optics_performance_parser.add_argument('input-files-pattern', help='A glob pattern for input files.')
    optics_performance_parser.add_argument('--output', help='The output file name without extension.')

    bundle_parser = subparsers.add_parser('jobs-performance')
    bundle_parser.add_argument('input', help='The input file.')
    bundle_parser.add_argument('--output', help='The output file name without extension.')
    bundle_parser.add_argument('--max-jobs', help='Maximum number of transfer jobs considered.')

    solution_parser = subparsers.add_parser('solution')
    solution_parser.add_argument('solution_input', help='The input file with solution.')
    solution_parser.add_argument('model_input', help='The input file with the model.')
    solution_parser.add_argument('elevation_input', help='The input file with elevation angles.')
    solution_parser.add_argument('key_rate_input', help='The input file with key rates.')
    solution_parser.add_argument('--output', help='The output file name without extension.')
    solution_parser.add_argument('--min-jobs',
                                 type=int,
                                 default=0,
                                 help='Filter out solutions with smaller number of transfer actions')

    export_parser = subparsers.add_parser('export')
    export_parser.add_argument('solution_input', help='The input file with solution.')
    export_parser.add_argument('forecast_input', help='The input file with weather forecast.')
    export_parser.add_argument('--output', help='The output file name without extension.')

    long_term_performance_parser = subparsers.add_parser('long-term-performance')
    long_term_performance_parser.add_argument('--data-dir')
    long_term_performance_parser.add_argument('--solution-dir')

    week_performance_parser = subparsers.add_parser('week-performance')
    week_performance_parser.add_argument('problem-file', type=pathlib.Path)
    week_performance_parser.add_argument('solution-file', type=pathlib.Path)

    aggregate_parser = subparsers.add_parser('aggregate')
    aggregate_parser.add_argument('--data-dir')

    bottleneck_parser = subparsers.add_parser('bottleneck')
    bottleneck_parser.add_argument('--data-dir')
    bottleneck_parser.add_argument('--min-run', default=0)

    service_level_parser = subparsers.add_parser('service-level')
    service_level_parser.add_argument('--data-dir')
    service_level_parser.add_argument('--solution-dir')

    subparsers.add_parser('all-service-level')

    network_traffic_parser = subparsers.add_parser('network-traffic')
    network_traffic_parser.add_argument('--solution-dir')
    network_traffic_parser.add_argument('--data-dir')

    communication_window_parser = subparsers.add_parser('communication-window')
    communication_window_parser.add_argument('--data-dir')
    communication_window_parser.add_argument('--station')

    weights_disturbed_parser = subparsers.add_parser('weights-disturbed')
    weights_disturbed_parser.add_argument('--data-dir')

    compare_parser = subparsers.add_parser('compare')
    compare_parser.add_argument('--problem')
    compare_parser.add_argument('--weather')
    compare_parser.add_argument('--baseline-solution')
    compare_parser.add_argument('--other-solution')

    key_rate_parser = subparsers.add_parser('key-rate')

    return parser.parse_args()


if __name__ == '__main__':
    args_ = parse_args()

    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['font.size'] = 12
    matplotlib.rcParams['font.family'] = 'sans-serif'
    # matplotlib.rcParams['font.sans-serif'] = ['Roboto']

    command = getattr(args_, 'command')
    if command == 'switch-time-performance':
        plot_switch_performance(args_)
    elif command == 'jobs-performance':
        plot_bundle(args_)
    # elif command == 'solution':
    #     plot_solution(args_)
    elif command == 'export':
        export_solution(args_)
    elif command == 'long-term-performance':
        plot_long_term_performance(args_)
    elif command == 'aggregate':
        plot_aggregate(args_)
    elif command == 'bottleneck':
        plot_bottleneck(args_)
    elif command == 'service-level':
        plot_service_level(args_)
    elif command == 'week-performance':
        plot_week_performance(args_)
    elif command == 'communication-window':
        plot_communication_window(args_)
    elif command == 'network-traffic':
        plot_network_traffic(args_)
    elif command == 'weights-disturbed':
        plot_weights_disturbed(args_)
    elif command == 'all-service-level':
        plot_all_service_levels(args_)
    elif command == 'compare':
        plot_compare(args_)
    elif command == 'key-rate':
        plot_key_rate(args_)
