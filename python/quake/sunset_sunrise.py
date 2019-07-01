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

import re
import datetime

import pytz

import quake.city


class SunsetSunriseIndex:
    YEAR_PATTERN = re.compile('^(?P<year>\d+) Sunrise and Sunset times for (?P<city>\w+)')
    FIRST_6MONTHS_PATTERN = re.compile('^January\s+February\s+March\s+April\s+May\s+June$')
    SECOND_6MONTHS_PATTERN = re.compile('^July\s+August\s+September\s+October\s+November\s+December$')
    DATA_PATTERN = re.compile(r'^(?P<day_of_month>\d+)\s{3}(?P<capture>[\d\s]+)$')
    TIME_PATTERN = re.compile(
        r'^(?P<rise>(?P<rise_hour>\d{2})\s(?P<rise_minute>\d{2}))\s{2}(?P<set>(?P<set_hour>\d{2})\s(?P<set_minute>\d{2}))')
    SEPARATOR_PATTERN = re.compile(r'^\s{3}')
    BLANK_PATTERN = re.compile(r'^\s{12}')
    BRITISH_LOCAL_TIME = pytz.timezone('Europe/London')

    def __init__(self, index):
        self.__index = index

    def is_night(self, city, date_time):
        sunset = self.__index[city][1][date_time.date()]
        if sunset <= date_time.time():
            return True

        next_date = date_time + datetime.timedelta(hours=24)
        sunrise = self.__index[city][0][next_date.date()]
        return date_time.time() <= sunrise

    def sunset(self, city, date):
        return self.__index[city][1][date]

    def sunrise(self, city, date):
        return self.__index[city][0][date]

    @staticmethod
    def from_files(file_paths):

        def load_file(file_path):
            day = None
            date = None
            month_offset = None
            year = None
            city = None
            sunsets = dict()
            sunrises = dict()

            with open(file_path, 'r') as input_stream:
                file_it = iter(input_stream)
                for raw_line in file_it:
                    line = raw_line.strip()
                    if not line:
                        continue

                    match = SunsetSunriseIndex.YEAR_PATTERN.match(line)
                    if match:
                        year = int(match.group('year'))
                        city = quake.city.City.from_name(match.group('city'))
                        continue

                    match = SunsetSunriseIndex.FIRST_6MONTHS_PATTERN.match(line)
                    if match:
                        month_offset = 0
                        continue

                    match = SunsetSunriseIndex.SECOND_6MONTHS_PATTERN.match(line)
                    if match:
                        month_offset = 6
                        continue

                    month_increment = 1
                    match = SunsetSunriseIndex.DATA_PATTERN.match(line)
                    if match:
                        day = int(match.group('day_of_month'))
                        capture = match.group('capture')
                        working_capture = capture
                        while working_capture:
                            time_match = SunsetSunriseIndex.TIME_PATTERN.match(working_capture)
                            if time_match:
                                date = datetime.date(year, month_offset + month_increment, day)

                                rise_hour = int(time_match.group('rise_hour'))
                                rise_minute = int(time_match.group('rise_minute'))
                                rise_time = datetime.time(rise_hour,
                                                          rise_minute)
                                rise_date_time = datetime.datetime.combine(date, rise_time)
                                rise_date_time_local = SunsetSunriseIndex.BRITISH_LOCAL_TIME.localize(rise_date_time)
                                rise_date_time_utc = rise_date_time_local.astimezone(pytz.utc)
                                sunrises[date] = rise_date_time_utc.time()

                                set_hour = int(time_match.group('set_hour'))
                                set_minute = int(time_match.group('set_minute'))
                                set_time = datetime.time(set_hour, set_minute)
                                set_date_time = datetime.datetime.combine(date, set_time)
                                set_date_time_local = SunsetSunriseIndex.BRITISH_LOCAL_TIME.localize(set_date_time)
                                set_date_time_utc = set_date_time_local.astimezone(pytz.utc)
                                sunsets[date] = set_date_time_utc.time()

                                working_capture = working_capture[time_match.span()[1]:]
                                continue

                            blank_match = SunsetSunriseIndex.BLANK_PATTERN.match(working_capture)
                            if blank_match:
                                working_capture = working_capture[blank_match.span()[1]:]
                                continue

                            separator_match = SunsetSunriseIndex.SEPARATOR_PATTERN.match(working_capture)
                            if separator_match:
                                working_capture = working_capture[separator_match.span()[1]:]
                                month_increment += 1
                                continue

            return city, sunrises, sunsets

        city_sunrises_sunsets = dict()
        for file_path in file_paths:
            city, sunrises, sunsets = load_file(file_path)
            city_sunrises_sunsets[city] = sunrises, sunsets

        return SunsetSunriseIndex(city_sunrises_sunsets)
