#!/usr/bin/env python3

import argparse
import json
import os.path
import datetime
import collections
import copy
import csv

import pandas
import numpy
import matplotlib.pyplot
import matplotlib.ticker


class WeatherForecast:
    def __init__(self, index):
        self.__index = index

        if self.__index:
            self.__start_time = min([min(message_index.keys()) for city_name, message_index in self.__index.items()])
        else:
            self.__start_time = datetime.datetime.max

    @staticmethod
    def columns():
        return ['city', 'date_time', 'cloud_coverage']

    @staticmethod
    def from_logs(logs):
        if not logs:
            return dict()

        index = dict()
        for city_log in logs:
            if 'city' not in city_log or 'name' not in city_log['city'] or 'list' not in city_log:
                continue

            city = city_log['city']['name']
            messages = []
            for message in city_log['list']:
                if 'dt' not in message:
                    continue
                messages.append((datetime.datetime.utcfromtimestamp(message['dt']), message))
            messages.sort(key=lambda m: m[0])

            message_index = collections.OrderedDict()
            for date_time, message in messages:
                message_index[date_time] = message
            index[city] = message_index
        return WeatherForecast(index)

    def to_records(self):
        records = []
        for city, message_index in self.__index.items():
            for date_time, message in message_index.items():
                records.append([city, date_time, message['clouds']['all']])
        return records

    def merge(self, other):
        master = max([self, other], key=lambda o: o.start_time)
        slave = min([self, other], key=lambda o: o.start_time)

        master_index = copy.deepcopy(master.__index)
        slave_index = copy.deepcopy(slave.__index)

        for first_level_key in slave_index.keys():
            if first_level_key not in slave_index:
                slave_index[first_level_key] = slave_index[first_level_key]

        for first_level_key in master_index.keys():
            if first_level_key not in slave_index:
                continue
            for second_level_key, message in slave_index[first_level_key].items():
                if second_level_key not in master_index[first_level_key]:
                    master_index[first_level_key][second_level_key] = message
        return WeatherForecast(master_index)

    @property
    def values(self):
        return self.__index

    @property
    def start_time(self):
        return self.__start_time


def parse_args():
    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers(dest='command')

    plot_parser = subparsers.add_parser('plot')
    plot_parser.add_argument('--input', required=True)
    plot_parser.add_argument('--output', required=True)

    print_parser = subparsers.add_parser('extract')
    print_parser.add_argument('--input', required=True)
    print_parser.add_argument('--output', required=True)
    print_parser.add_argument('--case', default='nominal', choices=['nominal', 'worst-case', 'best-case'])

    return parser.parse_args()


def load_forecasts(input_dir):
    forecasts = []
    for file_item in os.listdir(input_dir):
        file_path = os.path.join(input_dir, file_item)
        if not os.path.isfile(file_path):
            continue

        with open(file_path, 'r') as input_stream:
            logs = json.load(input_stream)
            forecasts.append(WeatherForecast.from_logs(logs))
    forecasts.sort(key=lambda item: item.start_time, reverse=True)

    if not forecasts:
        master = WeatherForecast(dict())
    else:
        master = forecasts[0]
        for forecast in forecasts[1:]:
            master = master.merge(forecast)
    return master


def plot_command(args):
    input_dir = getattr(args, 'input')
    output_file = getattr(args, 'output')

    forecast = load_forecasts(input_dir)
    data_frame = pandas.DataFrame.from_records(data=forecast.to_records(), columns=forecast.columns())
    heat_data_frame = data_frame.pivot_table(values='cloud_coverage', index='date_time', columns='city')

    figure, axis = matplotlib.pyplot.subplots()
    heat_map = axis.pcolor(heat_data_frame.values, cmap='binary')
    figure.colorbar(heat_map, ax=axis)

    axis.set_xticks(numpy.arange(len(heat_data_frame.columns.values)) + 0.5)
    axis.set_xticklabels(heat_data_frame.columns.values)

    values = [value.to_pydatetime() for value in heat_data_frame.index]

    def y_axis_label_formatter(value, pos=None):
        index = int(value)
        if index >= len(values):
            return ''
        return values[index].strftime('%Y-%m-%d %M:%H')

    axis.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(y_axis_label_formatter))
    matplotlib.pyplot.title('Cloud Cover Forecast')
    matplotlib.pyplot.setp(axis.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")
    figure.tight_layout()
    matplotlib.pyplot.savefig(output_file)


def extract_command(args):
    input_dir = getattr(args, 'input')
    output_file = getattr(args, 'output')

    def worst_case(value):
        return max(value, 90)

    def best_case(value):
        return min(value, 0)

    def nominal(value):
        return value

    case = getattr(args, 'case')
    if case == 'nominal':
        wrap_value = nominal
    elif case == 'worst-case':
        wrap_value = worst_case
    elif case == 'best-case':
        wrap_value = best_case
    else:
        raise ValueError('Unknown case option: %s'.format(case))

    forecast = load_forecasts(input_dir)
    rows = []
    for city, city_index in forecast.values.items():
        for date_time, message in city_index.items():
            rows.append({
                'city': city,
                'date_time': date_time,
                'cloud_cover': wrap_value(message['clouds']['all'])
            })

    with open(output_file, 'w') as output_stream:
        writer = csv.DictWriter(output_stream, ['city', 'date_time', 'cloud_cover'])
        writer.writeheader()
        writer.writerows(rows)


if __name__ == '__main__':
    """
    program plot --input='./data/forecasts' --output=cloud_cover.png
    program extract --input='./data/forecasts' --output=cloud_cover.csv
    """

    args = parse_args()
    command = getattr(args, 'command')
    if command == 'plot':
        plot_command(args)
    elif command == 'extract':
        extract_command(args)
