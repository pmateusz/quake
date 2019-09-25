#!/usr/bin/env python3
#
# Copyright 2018 Mateusz Polnik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
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
import logging
import math
import operator
import os
import os.path
import re
import random
import subprocess
import warnings
import pathlib
import typing

import tqdm

import pandas

import quake.city
import quake.util


class Settings:

    def __init__(self):
        self.__cp_solver_path = '/home/pmateusz/dev/quake/build/quake-main'
        self.__mip_solver_path = '/home/pmateusz/dev/quake/build/quake-mip'
        # self.__solver_path = '/home/pmateusz/dev/quake/cmake-build-debug/quake-main'
        self.__forecast_path = '/home/pmateusz/dev/quake/data/forecasts/ground_station_data_set_filled.csv'
        self.__forecast_frame_path = '/home/pmateusz/dev/quake/.data/ground_station_data_set.hdf'

    @property
    def cp_solver(self):
        return self.__cp_solver_path

    @property
    def mip_solver(self):
        return self.__mip_solver_path

    @property
    def forecast(self):
        return self.__forecast_path

    @property
    def forecast_frame(self):
        return self.__forecast_frame_path


settings = Settings()


def build_parser():
    parser = argparse.ArgumentParser()

    sub_parsers = parser.add_subparsers(dest='command')

    generate_command = sub_parsers.add_parser('generate')
    generate_command.add_argument('--year', default='2018')
    generate_command.add_argument('--output', default='./output')

    disturb_command = sub_parsers.add_parser('disturb')
    disturb_command.add_argument('--input')
    disturb_command.add_argument('--output')

    run_command = sub_parsers.add_parser('run')
    run_command.add_argument('--problem-dir', required=True)
    run_command.add_argument('--solution-prefix', default='solution')
    run_command.add_argument('--time-step', default=datetime.timedelta(seconds=30), action=quake.util.ParseTimeDeltaAction)
    run_command.add_argument('--gap-limit', default=0.05, type=float)

    elevation_parser = sub_parsers.add_parser('elevation')
    elevation_parser.add_argument('--data-dir', required=True)

    return parser


def get_cloud_cover_data_frame():
    if os.path.isfile(settings.forecast_frame):
        return pandas.read_hdf(settings.forecast_frame)
    else:
        DATETIME_INDEX = 1
        CITY_INDEX = 2
        CLOUD_COVER_INDEX = 23

        records = []
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
            with open(settings.forecast, 'r') as input_stream:
                dialect = csv.Sniffer().sniff(input_stream.read(4096))
                reader = csv.reader(input_stream, dialect=dialect)
                input_stream.seek(0)

                reader_it = iter(reader)
                next(reader_it)
                for line in reader_it:
                    records.append((datetime.datetime.strptime(line[DATETIME_INDEX], '%Y-%m-%d %H:%M:%S %z %Z'),
                                    quake.city.from_name(line[CITY_INDEX]).name,
                                    int(line[CLOUD_COVER_INDEX])))

        record_index = pandas.MultiIndex.from_tuples([(record[0], record[1]) for record in records],
                                                     names=['Time', 'City'])
        cloud_cover = [record[2] for record in records]
        frame = pandas.DataFrame(index=record_index, columns=['CloudCover'], data=cloud_cover)
        frame.to_hdf(settings.forecast_frame, key='cloud_cover')
        return frame


def old_generate(args):
    year = int(getattr(args, 'year'))
    raw_output_dir = getattr(args, 'output')
    output_dir = os.path.abspath(raw_output_dir)

    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    first_day_of_year = datetime.datetime(year=year, month=1, day=1)
    first_day_of_next_year = datetime.datetime(year=year + 1, month=1, day=1)
    first_day_of_week = first_day_of_year
    last_day_of_week = first_day_of_year + datetime.timedelta(days=6 - first_day_of_year.weekday())

    bunches = []
    while True:

        if first_day_of_next_year < last_day_of_week:
            last_day_of_week = first_day_of_next_year
            bunches.append((first_day_of_week, last_day_of_week))
            break

        bunches.append((first_day_of_week, last_day_of_week))
        first_day_of_week = last_day_of_week + datetime.timedelta(days=1)
        last_day_of_week = first_day_of_week + datetime.timedelta(days=6)

    full_frame = get_cloud_cover_data_frame()
    full_frame.sort_index(inplace=True)
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
        for start_day, end_day in tqdm.tqdm(bunches, desc='Generating problems', leave=False):
            try:
                def cmd_format_date_time(date_time):
                    return date_time.strftime('%Y-%m-%d')

                if start_day.year < end_day.year:
                    effective_end_night = end_day
                else:
                    effective_end_night = end_day + datetime.timedelta(days=1, hours=12)
                begin_stamp = pandas.to_datetime(start_day)
                end_stamp = pandas.to_datetime(effective_end_night)
                cloud_cover_slice = full_frame.loc[pandas.IndexSlice[begin_stamp: end_stamp, :]]

                with open(os.path.join(output_dir, 'cloud_cover_{0}.csv'.format(cmd_format_date_time(start_day))),
                          'w') as output_stream:
                    writer = csv.DictWriter(output_stream, fieldnames=['city', 'date_time', 'cloud_cover'])
                    writer.writeheader()
                    for index, content in cloud_cover_slice.iterrows():
                        writer.writerow({'city': index[0],
                                         'date_time': index[1],
                                         'cloud_cover': content['CloudCover']})

                process = subprocess.run(
                    [settings.solver,
                     '--command=minizinc',
                     '--first_day={0}'.format(cmd_format_date_time(start_day)),
                     '--last_day={0}'.format(cmd_format_date_time(effective_end_night)),
                     '--time_step=1',
                     '--output=' + os.path.join(output_dir, 'week_{0}.dzn'.format(cmd_format_date_time(start_day))),
                     '--convert-float-to-int',
                     '--v=1'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
                process.check_returncode()
            except:
                logging.exception('Failed to process work item {0} {1}', start_day, end_day)


def get_records(data_dir, prefix) -> typing.Dict[datetime.date, typing.Dict[str, pathlib.Path]]:
    records = {}
    for file_item in os.listdir(data_dir):
        match = re.match(r'^(?P<prefix>\w+?)_(?P<date>\d+-\d+-\d+).*?\..+$', file_item)
        if not match:
            continue
        date = match.group('date')
        local_prefix = match.group('prefix')

        if date not in records:
            records[date] = dict()

        if local_prefix == prefix:
            records[date][prefix] = pathlib.Path(os.path.join(data_dir, file_item))
    return records


def old_run_sequence(args):
    data_dir = getattr(args, 'data_dir')
    data_dir = os.path.abspath(data_dir)

    problem_dir = getattr(args, 'problem_dir')
    problem_dir = os.path.abspath(problem_dir)

    solution_prefix = getattr(args, 'solution_prefix')

    problem_records = get_records(problem_dir, 'week')
    cloud_cover_records = get_records(data_dir, 'cloud_cover')
    dates = set(problem_records.keys())
    dates.update(cloud_cover_records.keys())
    dates = sorted(dates)

    for date in dates:
        if 'cloud_cover' not in cloud_cover_records[date]:
            logging.warning("Missing 'cloud_cover' record for {0}", date)

        if 'week' not in problem_records[date]:
            logging.warning("Missing 'week' record for {0}", date)

    def generate_solution_file(given_date):
        return os.path.abspath(solution_prefix + '_{0}.json'.format(given_date))

    def run(date, problem_input_file, cloud_cover_input_file, solution_file, previous_solution_input_file=None):
        # process_args = [settings.solver,
        #                 '--command=solve-inventory-cp',
        #                 '--failure-scaling-factor=1000',
        #                 '--input={0}'.format(problem_input_file),
        #                 '--cloud-cover={0}'.format(cloud_cover_input_file),
        #                 '--output={0}'.format(solution_file),
        #                 '--objective-function=surplus-envelope']

        process_args = [settings.mip_solver,
                        '--input={0}'.format(problem_input_file),
                        '--cloud-cover={0}'.format(cloud_cover_input_file),
                        '--output={0}'.format(solution_file),
                        '--time-step=00:00:05']

        if previous_solution_input_file:
            if not os.path.isfile(previous_solution_input_file):
                logging.fatal('Previous solution file %s does not exist', previous_solution_input_file)
            process_args.append('--previous-solution=' + previous_solution_input_file)

        try:
            with open('out_{0}.log'.format(date), 'w') as output_stream:
                with open('err_{0}.log'.format(date), 'w') as error_stream:
                    process = subprocess.run(process_args, stdout=output_stream, stderr=error_stream)
                    process.check_returncode()
        except:
            logging.exception('Failed to process problem %s. Reproduction arguments: %s', date, process_args)

    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
        prev_solution_file = None
        for date in tqdm.tqdm(dates, desc='Processing day', leave=False):
            solution_file = generate_solution_file(date)
            if os.path.isfile(solution_file):
                prev_solution_file = solution_file
                continue

            run(date, problem_records[date]['week'], cloud_cover_records[date]['cloud_cover'], solution_file,
                prev_solution_file)
            prev_solution_file = solution_file


def run_mip_solver(problem_date: datetime.date,
                   problem_input_file: pathlib.Path,
                   solution_output_file: pathlib.Path,
                   time_step: datetime.timedelta,
                   gap_limit: float,
                   previous_solution_input_file: pathlib.Path = None) -> None:
    process_args = [settings.mip_solver,
                    '--input={0}'.format(problem_input_file),
                    '--output={0}'.format(solution_output_file),
                    '--time-step={0}'.format(time_step),
                    '--gap-limit={0}'.format(gap_limit)]

    if previous_solution_input_file:
        if not previous_solution_input_file.exists():
            logging.fatal('Previous solution file %s does not exist', previous_solution_input_file)
        process_args.append('--previous-solution={0}'.format(previous_solution_input_file))

    try:
        with open('out_{0}.log'.format(problem_date), 'w') as output_stream:
            with open('err_{0}.log'.format(problem_date), 'w') as error_stream:
                process = subprocess.run(process_args, stdout=output_stream, stderr=error_stream)
                process.check_returncode()
    except:
        logging.exception('Failed to process problem %s. Reproduction arguments: %s', problem_date, process_args)


def run_sequence(args) -> None:
    problem_dir = getattr(args, 'problem_dir')
    problem_dir = pathlib.Path(os.path.abspath(problem_dir))

    solution_prefix = getattr(args, 'solution_prefix')

    time_step = getattr(args, 'time_step')
    gap_limit = getattr(args, 'gap_limit')

    problem_records = get_records(problem_dir, 'week')
    problem_dates = list(problem_records.keys())
    problem_dates.sort()

    def get_solution_file_path(given_date) -> pathlib.Path:
        return pathlib.Path(os.path.abspath(solution_prefix + '_{0}.json'.format(given_date)))

    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
        prev_solution_file = None
        for problem_date in tqdm.tqdm(problem_dates, desc='Processing date', leave=False):
            solution_file = get_solution_file_path(problem_date)
            if solution_file.is_file():
                # warm starting simulation from last failure
                prev_solution_file = solution_file
                continue

            run_mip_solver(problem_date, problem_records[problem_date]['week'], solution_file, time_step, gap_limit, prev_solution_file)
            prev_solution_file = solution_file


def elevation(args):
    data_dir = getattr(args, 'data_dir')
    data_dir = os.path.abspath(data_dir)

    records = get_records(data_dir, 'week')

    def generate_elevation_file(given_date):
        return os.path.abspath('elevation_{0}.csv'.format(given_date.strftime('%Y-%m-%d')))

    def run_elevation(begin_date_time, end_date_time, elevation_file):
        process_args = [settings.solver,
                        '--command=elevation',
                        '--first-day=' + begin_date_time.strftime('%Y-%m-%d'),
                        '--last-day=' + end_date_time.strftime('%Y-%m-%d'),
                        '--output=' + elevation_file]

        try:
            with open('out_elevation_{0}.log'.format(date), 'w') as output_stream:
                with open('err_elevation_{0}.log'.format(date), 'w') as error_stream:
                    process = subprocess.run(process_args, stdout=output_stream, stderr=error_stream)
                    process.check_returncode()
        except:
            logging.exception('Failed to process problem %s. Reproduction arguments: %s', date, process_args)

    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
        time_step = datetime.timedelta(days=7)
        dates = sorted(records.keys())
        for date in tqdm.tqdm(dates, desc='Processing day', leave=False):
            begin_date_time = datetime.datetime.strptime(date, '%Y-%m-%d')
            end_date_time = begin_date_time + time_step

            if begin_date_time.year != end_date_time.year:
                end_date_time = datetime.datetime(year=begin_date_time.year + 1, month=1, day=1)

            elevation_file = generate_elevation_file(begin_date_time)
            run_elevation(begin_date_time, end_date_time, elevation_file)


def disturb(args):
    raw_input_dir = getattr(args, 'input')
    raw_output_dir = getattr(args, 'output')

    initial_edges = [
        (quake.city.LONDON, quake.city.GLASGOW, 0.04),
        (quake.city.LONDON, quake.city.CAMBRIDGE, 0.04),
        (quake.city.LONDON, quake.city.BRISTOL, 0.07),
        (quake.city.LONDON, quake.city.IPSWICH, 0.1),
        (quake.city.GLASGOW, quake.city.THURSO, 0.01),
        (quake.city.GLASGOW, quake.city.YORK, 0.02),
        (quake.city.GLASGOW, quake.city.BIRMINGHAM, 0.04),
        (quake.city.GLASGOW, quake.city.MANCHESTER, 0.01),
        (quake.city.MANCHESTER, quake.city.YORK, 0.03),
        (quake.city.MANCHESTER, quake.city.BRISTOL, 0.03),
        (quake.city.MANCHESTER, quake.city.BIRMINGHAM, 0.05),
        (quake.city.BIRMINGHAM, quake.city.IPSWICH, 0.05),
        (quake.city.BIRMINGHAM, quake.city.CAMBRIDGE, 0.01)
    ]

    def __get_local_weights(edges):
        local_weights = collections.defaultdict(float)
        for city_a, city_b, edge_weight in edges:
            local_weights[city_a] += edge_weight
            local_weights[city_b] += edge_weight
        return dict(local_weights)

    def __disturb(edges, fraction):
        cities = set()
        for city, _, _ in edges:
            cities.add(city)
        for _, city, _ in edges:
            cities.add(city)

        old_local_weights = __get_local_weights(edges)
        assert sum(old_local_weights.values()) >= 0.99

        raw_new_edges = []
        for city_a, city_b, old_weight in edges:
            new_weight = old_weight + old_weight * fraction * random.gauss(mu=0, sigma=1)
            raw_new_edges.append((city_a, city_b, new_weight))

        sum_raw_new_weights = sum(2 * w for _, _, w in raw_new_edges)
        assert sum_raw_new_weights > 0

        normalisation_factor = 1.0 / sum_raw_new_weights
        new_edges = []
        for city_a, city_b, edge_weight in raw_new_edges:
            new_edges.append((city_a, city_b, edge_weight * normalisation_factor))

        new_local_weights = __get_local_weights(new_edges)
        assert sum(new_local_weights.values()) > 0.99

        return new_local_weights

    fraction = 0.1

    initial_weights = __get_local_weights(initial_edges)
    new_weights = __disturb(initial_edges, fraction)

    input_dir = os.path.abspath(raw_input_dir)
    output_dir = os.path.abspath(raw_output_dir)

    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    problem_files = []
    problem_matcher = re.compile(r'^week_(?P<year>\d+)-(?P<month>\d+)-(?P<day>\d+)\.dzn$')
    for file_item in os.listdir(input_dir):
        match = problem_matcher.match(file_item)
        if not match:
            continue

        problem_date = datetime.date(int(match.group('year')), int(match.group('month')), int(match.group('day')))
        problem_path = os.path.join(input_dir, file_item)
        problem_files.append((problem_date, problem_path))

    problem_files.sort(key=operator.itemgetter(0))

    # get first solution, work out weights, work out consumption, work out initial buffer and order of stations
    problem_file = problem_files[0][1]
    with open(problem_file, 'r') as fp:
        content = fp.readlines()

    list_matcher = re.compile(r'^\w+ = \[(?P<content>[^\]]*)\];\s*$')
    set_matcher = re.compile(r'^\w+ = \{(?P<content>[^\}]*)\};\s*$')

    def get_list_of_numbers(line):
        match = list_matcher.match(line)
        if not match:
            return list()
        raw_content = match.group('content')
        content = list(map(float, raw_content.split(', ')))
        return content

    cities = []
    transfer_share = []
    for line in content:
        if line.startswith('transfer_share'):
            transfer_share = get_list_of_numbers(line)
        elif line.startswith('STATION'):
            match = set_matcher.match(line)
            if match:
                raw_content = match.group('content')
                names = [element.strip() for element in raw_content.split(', ')]
                cities = [quake.city.from_name(name) for name in names if name != 'None']

    assert transfer_share and cities

    updated_transfer_share = [0.0]
    for city_index, city in enumerate(cities, start=1):
        assert math.isclose(transfer_share[city_index], initial_weights[city])
        updated_transfer_share.append(new_weights[city])

    for problem_date, problem_file in problem_files:
        transfer_share = []
        transfer_share_index = -1
        initial_buffer = []
        initial_buffer_index = -1
        key_consumption = []
        key_consumption_index = -1

        with open(problem_file, 'r') as fp:
            content = fp.readlines()

        for index, line in enumerate(content):
            if line.startswith('transfer_share'):
                transfer_share_index = index
                transfer_share = get_list_of_numbers(line)
            elif line.startswith('initial_buffer'):
                initial_buffer_index = index
                initial_buffer = get_list_of_numbers(line)
            elif line.startswith('key_consumption'):
                key_consumption_index = index
                key_consumption = get_list_of_numbers(line)

        assert transfer_share and initial_buffer and key_consumption

        updated_initial_buffer = [0]
        for index in range(1, len(initial_buffer)):
            updated_initial_buffer.append(initial_buffer[index] * updated_transfer_share[index] / transfer_share[index])

        updated_key_consumption = [0]
        for index in range(1, len(key_consumption)):
            updated_key_consumption.append(
                key_consumption[index] * updated_transfer_share[index] / transfer_share[index])

        def get_string_from_list(iterable, key):
            return '{0} = [{1}];\n'.format(key, ', '.join(iterable))

        content[transfer_share_index] \
            = get_string_from_list(map(str, updated_transfer_share), 'transfer_share')
        content[initial_buffer_index] \
            = get_string_from_list(map(lambda v: str(int(v)), updated_initial_buffer), 'initial_buffer')
        content[key_consumption_index] \
            = get_string_from_list(map(lambda v: str(int(v)), updated_key_consumption), 'key_consumption')

        problem_output_file = 'week_{0}_fraction0{1}.dzn'.format(problem_date.strftime('%Y-%m-%d'), int(fraction * 100))
        problem_output_path = os.path.join(output_dir, problem_output_file)
        with open(problem_output_path, 'w') as fp:
            fp.writelines(content)


if __name__ == '__main__':
    parser_ = build_parser()

    args_ = parser_.parse_args()
    command = getattr(args_, 'command')

    if not command:
        parser_.print_help()
    elif command == 'generate':
        # old_generate(args_)
        pass
    elif command == 'run':
        run_sequence(args_)
    elif command == 'elevation':
        elevation(args_)
    elif command == 'disturb':
        # disturb(args_)
        pass
