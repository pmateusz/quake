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
import csv
import datetime
import operator

import quake.city


class CloudCoverIndex:

    def __init__(self, index, smooth: bool):
        self.__index = index
        self.__smooth = smooth

    def __call__(self, city: quake.city.City, time: datetime.datetime) -> float:
        if city not in self.__index:
            return 0.0

        time_records, cover_records = self.__index[city]

        if not time_records:
            return 0

        if len(time_records) == 1:
            return time_records[0]

        start_time = time_records[0]
        update_frequency = time_records[1] - time_records[0]

        time_from_start = time - start_time
        left_index = int(time_from_start.total_seconds() / update_frequency.total_seconds())

        assert 0 <= left_index

        if not self.__smooth:
            # implementation must be consistent with ./quake/src/main/forecast.h
            return cover_records[left_index]

        right_index = left_index + 1
        if right_index >= len(cover_records):
            return cover_records[left_index]

        left_distance = (time - time_records[left_index]).total_seconds() / update_frequency.total_seconds()
        right_distance = (time_records[right_index] - time).total_seconds() / update_frequency.total_seconds()

        assert right_distance >= 0
        assert left_distance >= 0

        if left_distance <= right_distance:
            return cover_records[left_index]
        else:
            return cover_records[right_index]

        # compute time distance from both windows

        # # old behaviour
        # import bisect
        # time_records, cover_records = self.__index[city]
        # right_index = bisect.bisect_left(time_records, time)
        # if right_index == 0 or right_index >= len(time_records):
        #     return 0
        #
        # left_index = right_index - 1
        # left_time, right_time = time_records[left_index], time_records[right_index]
        # assert left_time <= time <= right_time
        #
        # left_delta, right_delta = time - left_time, right_time - time
        # left_cover, right_cover = cover_records[left_index], cover_records[right_index]
        # if left_delta < right_delta:
        #     return left_cover
        # else:
        #     return right_cover
        # return left_cover

    @staticmethod
    def from_csv(file_path):
        interim_station_index = collections.defaultdict(list)
        with open(file_path, 'r') as input_stream:
            reader = csv.reader(input_stream, dialect=csv.Sniffer().sniff(input_stream.read(4096)))
            input_stream.seek(0)
            next(reader)  # skip header
            for raw_time, raw_city, raw_cover in reader:
                city = quake.city.from_name(raw_city)
                interim_station_index[city].append(
                    (datetime.datetime.strptime(raw_time, '%Y-%m-%d %H:%M:%S'), float(raw_cover)))

        city_index = dict()
        for city, records in interim_station_index.items():
            records.sort(key=operator.itemgetter(0))
            city_index[city] = (list(map(operator.itemgetter(0), records)), list(map(operator.itemgetter(1), records)))

        return CloudCoverIndex(index=city_index)

    @staticmethod
    def merge(indices):
        interim_station_index = collections.defaultdict(list)
        for local_index in indices:
            for station in local_index.__index:
                interim_station_index[station].extend(list(zip(*local_index.__index[station])))
        station_index = dict()
        for station, records in interim_station_index.items():
            records.sort(key=operator.itemgetter(0))
            station_index[station] = (list(map(operator.itemgetter(0), records)),
                                      list(map(operator.itemgetter(1), records)))
        return CloudCoverIndex(index=station_index)
