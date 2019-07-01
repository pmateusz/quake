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

import datetime


def parse_datetime(value):
    return datetime.datetime.strptime(value, '%Y-%b-%d %H:%M:%S')


def copy_if_set(target, source, target_property_name, source_property_name, transform=None):
    if source_property_name in source:
        value = source[source_property_name]
        value_to_use = transform(value) if transform else value
        setattr(target, target_property_name, value_to_use)


class BoundedValue:

    def __init__(self):
        self.upper_bound = None

    @staticmethod
    def from_json(document):
        value = BoundedValue()
        copy_if_set(value, document, 'upper_bound', 'upper_bound', int)
        return value


class Solution:
    class Station:
        def __init__(self):
            self.keys_transferred = None
            self.station = None

        @staticmethod
        def from_json(document):
            station = Solution.Job()
            copy_if_set(station, document, 'keys_transferred', 'key_rate', int)
            copy_if_set(station, document, 'station', 'station')
            return station

    class Job:
        def __init__(self):
            self.start_time = None
            self.end_time = None
            self.station = None
            self.keys_transferred = None

        @staticmethod
        def from_json(document):
            job = Solution.Job()
            copy_if_set(job, document, 'start_time', 'start_time', parse_datetime)
            copy_if_set(job, document, 'end_time', 'end_time', parse_datetime)
            copy_if_set(job, document, 'station', 'station')
            copy_if_set(job, document, 'keys_transferred', 'key_rate', int)
            return job

    def __init__(self):
        self.total_keys_transferred = None
        self.jobs = []
        self.stations = []

    def keys_transferred(self, station):
        for job in self.stations:
            if job.station == station.name:
                return job.keys_transferred
        return 0

    @staticmethod
    def from_json(document):
        solution = Solution()
        copy_if_set(solution, document, 'total_keys_transferred', 'total_key_rate', int)

        if 'actions' in document:
            solution.jobs = [Solution.Job.from_json(job) for job in document['actions']]

        if 'stations' in document:
            solution.stations = [Solution.Station.from_json(station) for station in document['stations']]

        return solution


class SolutionBundle:
    class Metadata(object):
        def __init__(self):
            self.dummy_station = None
            self.start_time = None
            self.end_time = None
            self.station_key_rate = BoundedValue()
            self.stations = []
            self.step_duration = None
            self.switch_duration = None
            self.total_keys_transferred = BoundedValue()

        @staticmethod
        def from_json(document):
            metadata = SolutionBundle.Metadata()
            copy_if_set(metadata, document, 'dummy_station', 'dummy_station')
            copy_if_set(metadata, document, 'end_time', 'end_time', parse_datetime)
            copy_if_set(metadata, document, 'start_time', 'start_time', parse_datetime)
            copy_if_set(metadata, document, 'stations', 'stations')
            copy_if_set(metadata, document, 'step_duration', 'step_duration', int)
            copy_if_set(metadata, document, 'switch_duration', 'switch_duration', int)
            copy_if_set(metadata, document, 'total_keys_transferred', 'total_key_rate', BoundedValue.from_json)
            copy_if_set(metadata, document, 'station_key_rate', 'station_key_rate', BoundedValue.from_json)
            return metadata

    def __init__(self, metadata, solutions):
        self.__metadata = metadata
        self.__solutions = solutions

    @property
    def metadata(self):
        return self.__metadata

    @property
    def solutions(self):
        return self.__solutions

    @staticmethod
    def from_json(document):
        metadata = None
        solutions = []

        if 'metadata' in document:
            metadata = SolutionBundle.Metadata.from_json(document['metadata'])

        if 'solutions' in document:
            solutions = [Solution.from_json(solution_document) for solution_document in document['solutions']]

        return SolutionBundle(metadata, solutions)
