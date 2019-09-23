#!/usr/bin/env python3

import argparse
import collections
import csv
import datetime
import json
import logging
import os
import warnings

import tqdm

INPUT_FILE_ARG = 'input_file'
INPUT_DELTA_ARG = 'delta'
YEAR_FILTER_ARG = 'year_filter'
DATE_TIME_ISO_INDEX = 1
LOCATION_INDEX = 2
TEMPERATURE_INDEX = 6


class InputFileValidator(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        if os.path.isfile(values):
            setattr(namespace, self.dest, os.path.abspath(os.path.expandvars(values)))
            return

        if os.path.exists(values):
            raise argparse.ArgumentError('Path {0} does not point to a file'.format(values))
        raise argparse.ArgumentError('File {0} does not exist'.format(values))


class TimeDeltaValidator(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        try:
            time_delta_date_time = datetime.datetime.strptime(values, '%H:%M:%S')
            time_delta = time_delta_date_time - datetime.datetime.combine(time_delta_date_time.date(), datetime.time())
            setattr(namespace, self.dest, time_delta)
        except ValueError:
            raise argparse.ArgumentError("Value '{0}' is not a valid time delta.".format(values))


def parse_args():
    parser = argparse.ArgumentParser(description='Checks OpenWeatherMap data sets for consistency')
    parser.add_argument(INPUT_FILE_ARG, action=InputFileValidator, help='A data set file to check consistency')
    parser.add_argument('--' + INPUT_DELTA_ARG,
                        action=TimeDeltaValidator,
                        help='Maximum time period of missing data',
                        default=datetime.timedelta(hours=1))
    parser.add_argument('--' + YEAR_FILTER_ARG, help='Filter warnings to the year')
    return parser.parse_args()


def load_csv(input_file):
    with open(input_file, 'r') as file_stream:
        dialect = csv.Sniffer().sniff(file_stream.read(4096))
        reader = csv.reader(file_stream, dialect=dialect)
        file_stream.seek(0)

        reader_it = iter(reader)
        next(reader_it, None)  # skip the header

        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)

            location_records = collections.defaultdict(list)
            for line in tqdm.tqdm(reader, desc='Loading data set', leave=False):
                current_location = int(line[LOCATION_INDEX])
                current_time = datetime.datetime.strptime(line[DATE_TIME_ISO_INDEX], '%Y-%m-%d %H:%M:%S %z %Z')
                location_records[current_location].append(current_time)
            return location_records


def load_json(input_file):
    with open(input_file, 'r') as file_stream:
        location_records = collections.defaultdict(list)
        data = json.load(file_stream)
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
            for record in tqdm.tqdm(data, desc='Loading data set', leave=False):
                current_time = datetime.datetime.strptime(record['dt_iso'], '%Y-%m-%d %H:%M:%S %z %Z')
                current_location = record['city_id']
                location_records[current_location].append(current_time)
        return location_records


def check_continuity(args):
    input_file_arg = getattr(args, INPUT_FILE_ARG)
    time_delta_arg = getattr(args, INPUT_DELTA_ARG)
    date_filter_arg = getattr(args, YEAR_FILTER_ARG, None)

    def accepting_filter(date_time):
        return False

    filter_function = accepting_filter

    if date_filter_arg:
        date_filter = int(date_filter_arg)
        time_zone = datetime.timezone(offset=datetime.timedelta())
        begin_date_filter = datetime.datetime(year=date_filter, month=1, day=1, tzinfo=time_zone)
        end_date_filter = datetime.datetime(year=date_filter, month=12, day=31, hour=23, minute=59, second=59, tzinfo=time_zone)
        filter_function = lambda date_time: date_time <= begin_date_filter or end_date_filter <= date_time

    _, ext = os.path.splitext(input_file_arg)
    if ext == '.csv':
        location_records = load_csv(input_file_arg)
    elif ext == '.json':
        location_records = load_json(input_file_arg)
    else:
        raise ValueError("Unsupported file extension '{0}'".format(ext))

    if not location_records:
        logging.warning('No records found')
        return

    for location, records in location_records.items():
        records.sort()

    min_time = min(records[0] for _, records in location_records.items())
    max_time = max(records[-1] for _, records in location_records.items())

    for location, records in location_records.items():
        if records[0] != min_time and not filter_function(records[0]):
            logging.warning('Truncated start - Location %s starts at time %s instead of %s', location, records[0], min_time)

        records_it = iter(records)
        last_time = next(records_it)
        for current_time in records_it:
            time_delta = current_time - last_time
            if time_delta > time_delta_arg and not (filter_function(current_time) or filter_function(last_time)):
                logging.warning('Missing data - Location %s has missing data between %s and %s', location, last_time, current_time)
            last_time = current_time

        if records[-1] != max_time and not filter_function(records[-1]):
            logging.warning('Truncated end - Location %s ends at time %s instead of %s', location, records[-1], max_time)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    args_ = parse_args()
    check_continuity(args_)
