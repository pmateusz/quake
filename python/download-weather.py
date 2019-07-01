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

import argparse
import collections
import logging
import json
import os
import sys
import requests
import datetime

CityLocator = collections.namedtuple('CityLocator',
                                     ['key', 'name', 'country_code', 'longitude', 'latitude'],
                                     verbose=False)

London = CityLocator(2643743, "London", "GB", -0.12574, 51.50853)
Birmingham = CityLocator(2655603, "Birmingham", "GB", -1.89983, 52.481419)
Glasgow = CityLocator(2648579, "Glasgow", "GB", -4.25763, 55.86515)
Thurso = CityLocator(2635881, "Thurso", "GB", -3.52594, 58.592709)
Manchester = CityLocator(2643123, "Manchester", "GB", -2.23743, 53.480949)
Bristol = CityLocator(2654675, "Bristol", "GB", -2.59665, 51.455231)
Ipswich = CityLocator(2646057, "Ipswich", "GB", 1.15545, 52.05917)
Cambridge = CityLocator(2653941, "Cambridge", "GB", 0.11667, 52.200001)
York = CityLocator(2633352, "York", "GB", -1.08271, 53.95763)


class QueryEngine:
    def __init__(self, api_key):
        self.__api_key = api_key
        self.__endpoint = 'https://api.openweathermap.org/data/2.5/forecast'
        self.__schema = ''

    def get_weather(self, city):
        request = requests.get(self.__endpoint, params={
            'APPID': self.__api_key,
            'id': city.key,
            'units': 'metric'
        })
        return request.json()


def load_api_key():
    settings_file_path = os.path.expanduser('~/dev/quake/.data/settings.json')
    with open(settings_file_path, 'r') as file_stream:
        settings = json.load(file_stream)
    return settings['weather_api_key']


def test_query():
    response_file = 'query.json'

    if not os.path.exists(response_file):
        engine = QueryEngine(load_api_key())
        response = engine.get_weather(London)
        with open(response_file, 'w') as output_stream:
            json.dump(response, output_stream)

    with open(response_file, 'r') as input_stream:
        response = json.load(input_stream)

    assert response['cod'] == '200'
    print(response)
    for entry in response['list']:
        print(datetime.datetime.utcfromtimestamp(entry['dt']), entry['clouds']['all'])
    print(response['city'])


def parse_args():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command_name')
    inspect_parser = subparsers.add_parser('inspect',
                                           help='Loads a weather forecast and prints information on the cloud coverage')
    inspect_parser.add_argument('forecast_file', help='File with the weather forecast')
    pull_parser = subparsers.add_parser('pull', help='Downloads weather information for all cities')
    pull_parser.add_argument('--output-dir', required=False)
    return parser.parse_args()


def pull(args):
    output_dir = getattr(args, 'output_dir')

    def serializer(obj):
        """JSON serializer for objects not serializable by default json code"""

        if isinstance(obj, (datetime.datetime, datetime.date)):
            return obj.isoformat()
        raise TypeError("Type %s is not serializable".format(type(obj)))

    forecast_file = 'forecast_' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M') + '.json'

    cities = [London, Birmingham, Glasgow, Thurso, Manchester, Bristol, Ipswich, Cambridge, York]
    engine = QueryEngine(load_api_key())
    responses = []
    for city in cities:
        response = engine.get_weather(city)
        if response['cod'] != '200':
            logging.critical('Failed to download the weather information for city %s due to error: %s',
                             city.name,
                             response)
            sys.exit(-1)
        response['download_time'] = datetime.datetime.now()
        responses.append(response)

    if output_dir:
        forecast_file_path = forecast_file_path = os.path.join(output_dir, forecast_file)
    else:
        forecast_file_path = os.path.abspath(forecast_file)

    try:
        final_forecast_file_path = forecast_file_path
        with open(final_forecast_file_path, 'w') as output_stream:
            json.dump(responses, output_stream, default=serializer)
    except BaseException:
        logging.error('Failed to save %s due to exception. Rollback to current directory.', forecast_file_path)

        # failed to save in a default location - fallback
        final_forecast_file_path = os.path.abspath(forecast_file)
        with open(final_forecast_file_path, 'w') as output_stream:
            json.dump(responses, output_stream, default=serializer)

    print('Forecast saved in file: {0}'.format(final_forecast_file_path))


def inspect(forecast_file_path):
    with open(forecast_file_path, 'r') as input_stream:
        weather_report = json.load(input_stream)
        for city_report in weather_report:
            print(city_report['city']['name'])
            for entry in city_report['list']:
                print(datetime.datetime.utcfromtimestamp(entry['dt']), entry['clouds']['all'])


if __name__ == '__main__':
    args = parse_args()
    command = getattr(args, 'command_name', None)
    if not command:
        print('Provide a valid command', file=sys.stderr)
        sys.exit(-1)

    if command == 'pull':
        pull(args)
    elif command == 'inspect':
        inspect(args.forecast_file)
    else:
        print('Unknown command: {0}'.format(command), file=sys.stderr)
        sys.exit(-1)
