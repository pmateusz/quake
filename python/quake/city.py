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


class City:

    def __init__(self, key, name, country_code, longitude, latitude):
        self.__key = key
        self.__name = name
        self.__country_code = country_code
        self.__longitude = longitude
        self.__latitude = latitude

    @property
    def key(self):
        return self.__key

    @property
    def name(self):
        return self.__name

    @property
    def country_code(self):
        return self.__country_code

    @property
    def longitude(self):
        return self.__longitude

    @property
    def latitude(self):
        return self.__latitude

    @staticmethod
    def from_key(value):
        return None

    @staticmethod
    def from_name(value):
        return None

    def __eq__(self, other):
        return isinstance(other, City) and self.key == other.key

    def __hash__(self):
        return self.key

    def __str__(self):
        return self.name

    def __lt__(self, other):
        assert isinstance(other, City)

        return self.name < other.name


NONE = City(0, "None", "", 0, 0)
BIRMINGHAM = City(2655603, "Birmingham", "GB", -1.89983, 52.481419)
BRISTOL = City(2654675, "Bristol", "GB", -2.59665, 51.455231)
CAMBRIDGE = City(2653941, "Cambridge", "GB", 0.11667, 52.200001)
GLASGOW = City(2648579, "Glasgow", "GB", -4.25763, 55.86515)
IPSWICH = City(2646057, "Ipswich", "GB", 1.15545, 52.05917)
LONDON = City(2643743, "London", "GB", -0.12574, 51.50853)
MANCHESTER = City(2643123, "Manchester", "GB", -2.23743, 53.480949)
THURSO = City(2635881, "Thurso", "GB", -3.52594, 58.592709)
YORK = City(2633352, "York", "GB", -1.08271, 53.95763)

__CITY_IDS = {
    NONE.key: NONE,
    BIRMINGHAM.key: BIRMINGHAM,
    BRISTOL.key: BRISTOL,
    CAMBRIDGE.key: CAMBRIDGE,
    GLASGOW.key: GLASGOW,
    IPSWICH.key: IPSWICH,
    LONDON.key: LONDON,
    MANCHESTER.key: MANCHESTER,
    THURSO.key: THURSO,
    YORK.key: YORK
}

__CITY_NAMES = {city.name.lower(): city for city in __CITY_IDS.values()}


def from_key(value):
    key = value
    if not isinstance(value, int):
        key = int(value)

    if key not in __CITY_IDS:
        raise ValueError('Key {0} is not a valid city id'.format(key))

    return __CITY_IDS[key]


def from_name(name):
    normalized_name = name.lower().strip()
    if normalized_name not in __CITY_NAMES:
        raise ValueError('Name {0} is not a valid city name'.format(name))
    return __CITY_NAMES[normalized_name]


City.from_key = staticmethod(from_key)
City.from_name = staticmethod(from_name)
