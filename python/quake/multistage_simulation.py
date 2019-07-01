#!/usr/bin/env python3
#
# Copyright 2019 Mateusz Polnik
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

import collections
import datetime
import operator
import os
import re
import math
import json

import pandas

import quake.minizinc
import quake.city
import quake.solution
import quake.transfer_rate

STATION_KEY = 'Station'
WEEK_KEY = 'Week'

WeeklyRecordColumns = [STATION_KEY, WEEK_KEY, 'KeyRatio', 'KeyBuffer', 'KeyConsumed', 'KeyTransferred', 'Duration']

WeeklyRecord = collections.namedtuple('WeeklyRecord', WeeklyRecordColumns)


class MultistageSimulation:

    def __init__(self, record_index, transfer_rates):
        self.__record_index = record_index
        self.__stations = [*set(quake.city.from_name(station)
                                for leaf_index in self.__record_index.values()
                                for station in leaf_index.keys())]
        self.__stations.sort(key=operator.attrgetter('name'))
        self.__transfer_rates = {}
        for station in self.__stations:
            self.__transfer_rates[station] = transfer_rates[station.name]

    def compute_bottlenecks(self):
        rows = []
        for date in sorted(self.__record_index.keys()):
            elements = list(self.__record_index[date].values())
            elements.sort(key=operator.attrgetter('KeyRatio'))
            rows.append({element.Station: index for index, element in enumerate(elements)})
        return rows

    def to_frame(self):
        rows = []
        for date, leaf_index in self.__record_index.items():
            for record in leaf_index.values():
                rows.append([record.Station.name, record.Week, record.KeyRatio, record.KeyBuffer, record.KeyConsumed,
                             record.KeyTransferred, record.Duration])

        data_frame = pandas.DataFrame(columns=WeeklyRecordColumns, data=rows)
        data_frame.sort_values([WEEK_KEY, STATION_KEY], inplace=True)
        return data_frame

    @property
    def stations(self):
        return self.__stations

    @property
    def transfer_rates(self):
        return self.__transfer_rates

    @staticmethod
    def load_from_dir(data_dir, solution_dir):
        solution_files = []
        for file_item in os.listdir(solution_dir):
            match = re.match(r'^solution(?:[^\d]*)?\d+_(?P<date>.*?)\.json$', file_item)
            if match:
                raw_date = match.group('date')
                date = datetime.datetime.strptime(raw_date, '%Y-%m-%d')
                solution_files.append((date, os.path.join(solution_dir, file_item)))
        solution_files.sort(key=operator.itemgetter(0))

        min_date = min(solution_files, key=operator.itemgetter(0))[0]
        min_problem_file_prefix = 'week_{0}'.format(min_date.date())

        min_problem_file = None
        for file_item in os.listdir(data_dir):
            if file_item.startswith(min_problem_file_prefix):
                min_problem_file = os.path.join(data_dir, file_item)
                break
        assert min_problem_file

        with open(min_problem_file, 'r') as input_file:
            min_problem = quake.minizinc.MiniZincLoader().load(input_file)

        bundles = []
        for date, solution_file in solution_files:
            with open(solution_file, 'r') as input_stream:
                document = json.load(input_stream)
                bundles.append(quake.solution.SolutionBundle.from_json(document))

        # apply transfer rate reduction from 128 to 256 keys
        bundles_to_use = [quake.transfer_rate.adjust_128_to_256(bundle) for bundle in bundles]
        del bundles

        # build data frame - date, station, received, transferred
        stations = {station for station in min_problem['STATION'] if station != 'None'}

        initial_buffer = {station: quake.transfer_rate.adjust_128_to_256(initial_buffer)
                          for station, initial_buffer
                          in zip(min_problem['STATION'], min_problem['initial_buffer'])
                          if station in stations}

        transfer_rate = {station: transfer_share
                         for station, transfer_share
                         in zip(min_problem['STATION'], min_problem['transfer_share'])
                         if station in stations}

        assert math.isclose(sum(transfer_rate.values()), 1.0, abs_tol=0.001)

        def key_ratio(buffers, station):
            return math.floor(buffers[station] / transfer_rate[station])

        def key_usage(buffers):
            min_ratio = min(key_ratio(buffers, station) for station in buffers.keys())
            keys_consumed = {station: int(math.floor(min_ratio * transfer_rate[station])) for station in buffers.keys()}

            for station in buffers.keys():
                assert buffers[station] >= keys_consumed[station]

            return keys_consumed

        def get_duration_in_days(solution):
            min_start_time = min(job.start_time for job in solution.jobs)
            max_start_time = max(job.start_time for job in solution.jobs)
            duration_in_days = math.ceil((max_start_time - min_start_time).total_seconds() / (60 * 60 * 24))
            return datetime.timedelta(days=duration_in_days)

        date_record = collections.defaultdict(dict)

        stations = initial_buffer.keys()

        # reshape records to avoid week of records, a record loaded from disk holds key rate
        # and key consumption from the beginning of a week whereas key buffer and key transferred
        # are registered at the end of the week
        prev_key_buffer = initial_buffer
        prev_transferred = {station: 0 for station in stations}
        dates = list(map(operator.itemgetter(0), solution_files))
        for current_date, current_bundle in zip(dates, bundles_to_use):
            current_solution = current_bundle.solutions[0]
            current_consumed = key_usage(prev_key_buffer)

            date_record[current_date] = {station: WeeklyRecord(Station=quake.city.from_name(station),
                                                               Week=current_date,
                                                               KeyRatio=key_ratio(prev_key_buffer, station),
                                                               KeyBuffer=prev_key_buffer[station],
                                                               KeyConsumed=current_consumed[station],
                                                               KeyTransferred=prev_transferred[station],
                                                               Duration=get_duration_in_days(current_solution))
                                         for station in initial_buffer.keys()}

            current_transferred = {job.station: job.keys_transferred for job in current_solution.stations
                                   if job.station in stations}
            current_key_buffer = {station: prev_key_buffer[station] - current_consumed[station]
                                           + current_transferred[station] for station in stations}

            prev_transferred = current_transferred
            prev_key_buffer = current_key_buffer

        return MultistageSimulation(date_record, transfer_rate)
