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

import os
import json
import datetime
import concurrent.futures
import logging
import warnings
import argparse

import pandas
import tqdm
import matplotlib.pyplot
import matplotlib.dates

import quake.city

# TODO: cache weather into a data frame

BUILD_CACHE_COMMAND = 'build-cache'
PLOT_COMMAND = 'plot-cloud-cover'
CACHE_FILE = 'forecast.hdf'
CACHE_FILE_TABLE = 'a'


def parse_args():
    parser = argparse.ArgumentParser()

    sub_parsers = parser.add_subparsers(dest='command')
    build_cache_parser = sub_parsers.add_parser(BUILD_CACHE_COMMAND)
    plot_parser = sub_parsers.add_parser(PLOT_COMMAND)
    plot_parser.add_argument('--from')

    return parser.parse_args()


def build_cache_command(args):
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
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            future_to_path = {executor.submit(load_data_frame, file_path): file_path for file_path in file_paths}
            for future in tqdm.tqdm(concurrent.futures.as_completed(future_to_path), total=len(future_to_path)):
                file_path = future_to_path[future]
                try:
                    data_frames.append(future.result())
                except Exception as ex:
                    logging.exception('%s generated error %s', file_path, ex)
    master_data_frame = pandas.concat(data_frames)
    master_data_frame.drop_duplicates(inplace=True)
    master_data_frame.to_hdf(CACHE_FILE, CACHE_FILE_TABLE)


def plot_command(args):
    from_arg = getattr(args, 'from')
    if from_arg:
        left_time_limit = datetime.datetime.strptime(from_arg, '%Y-%m-%d')
    else:
        left_time_limit = None

    master_data_frame = pandas.read_hdf(CACHE_FILE, CACHE_FILE_TABLE)
    if left_time_limit:
        master_data_frame = master_data_frame[master_data_frame['DateTime'] > left_time_limit].copy()

    cities = master_data_frame['City'].unique()
    for forecast_distance in [pandas.Timedelta(days=1), pandas.Timedelta(days=2),
                              pandas.Timedelta(days=3), pandas.Timedelta(days=4)]:
        for city in cities:
            forecast_hours = forecast_distance.total_seconds() / matplotlib.dates.SEC_PER_HOUR

            data_frame = master_data_frame[master_data_frame['City'] == city].copy()

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


if __name__ == '__main__':
    args = parse_args()
    command = getattr(args, 'command')

    if command == BUILD_CACHE_COMMAND:
        build_cache_command(args)
    elif command == PLOT_COMMAND:
        plot_command(args)
