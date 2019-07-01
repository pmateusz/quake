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

import bisect
import copy

import odf.opendocument
import odf.table

import pandas

import quake.solution

CONFIG_KEY = 'Config'
ELEVATION_KEY = 'Elevation'
KEY_RATE_KEY = 'KeyRate'
BIT_RATE_KEY = 'BitRate'
KEY_RATE_CONFIG = '633'

MIN_ANGLE = 10.0


def adjust_128_to_256(value):
    def __adjust_bundle_in_place(bundle):
        if bundle.metadata.total_keys_transferred.upper_bound:
            bundle.metadata.total_keys_transferred.upper_bound = adjust_128_to_256(bundle.metadata.total_keys_transferred.upper_bound)
        if bundle.metadata.station_key_rate.upper_bound:
            bundle.metadata.station_key_rate.upper_bound \
                = adjust_128_to_256(bundle.metadata.station_key_rate.upper_bound)

        for solution in bundle.solutions:
            solution.total_keys_transferred = adjust_128_to_256(solution.total_keys_transferred)
            for station_job in solution.stations:
                station_job.keys_transferred = adjust_128_to_256(station_job.keys_transferred)
            for local_job in solution.jobs:
                local_job.keys_transferred = adjust_128_to_256(local_job.keys_transferred)
        return bundle

    if isinstance(value, quake.solution.SolutionBundle):
        return __adjust_bundle_in_place(copy.deepcopy(value))

    if isinstance(value, int):
        return value // 2
    return value / 2


class TransferRateIndex:

    def __init__(self, angles, transfers):
        self.__angles = angles
        self.__transfers = transfers

    def __call__(self, elevation):
        if elevation < MIN_ANGLE:
            return 0.0

        position = bisect.bisect_left(self.__angles, elevation)
        if position != len(self.__angles) and position != 0:
            left_right_distance = self.__angles[position] - self.__angles[position - 1]
            left_part = (elevation - self.__angles[position - 1]) / left_right_distance
            right_part = (self.__angles[position] - elevation) / left_right_distance

            assert left_part >= 0.0
            assert right_part >= 0.0
            assert left_part + right_part >= 0.99

            return adjust_128_to_256(
                self.__transfers[position - 1] * left_part + self.__transfers[position] * right_part)

        if position == len(self.__angles):
            return adjust_128_to_256(self.__transfers[position - 1])

        assert position == 0
        return adjust_128_to_256(self.__transfers[0])

    @staticmethod
    def from_odf(file_path):
        spreadsheet = odf.opendocument.load(file_path)

        data = []
        sheets = spreadsheet.getElementsByType(odf.table.Table)
        for sheet in sheets:
            config = sheet.attributes[('urn:oasis:names:tc:opendocument:xmlns:table:1.0', 'name')]
            table_row_it = iter(sheet.getElementsByType(odf.table.TableRow))
            header_row = next(table_row_it, None)

            header_cells = [str(cell) for cell in header_row.getElementsByType(odf.table.TableCell)]
            if header_cells != ['Elevation', 'Rate of Secrete Keys', 'Rate of Secrete bits']:
                raise RuntimeError('Unexpected header names')

            for row in table_row_it:
                row_cells = row.getElementsByType(odf.table.TableCell)
                if len(row_cells) != 3:
                    continue
                data.append({
                    CONFIG_KEY: config,
                    ELEVATION_KEY: float(str(row_cells[0])),
                    KEY_RATE_KEY: float(str(row_cells[1])),
                    BIT_RATE_KEY: float(str(row_cells[2]))
                })

        frame = pandas.DataFrame(columns=[CONFIG_KEY, ELEVATION_KEY, KEY_RATE_KEY, BIT_RATE_KEY], data=data)
        filtered_frame = frame[frame[CONFIG_KEY] == KEY_RATE_CONFIG].copy()
        angles = list(filtered_frame[ELEVATION_KEY].values)
        transfers = list(filtered_frame[KEY_RATE_KEY].values)
        return TransferRateIndex(angles, transfers)
