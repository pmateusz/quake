import argparse
import concurrent.futures
import datetime
import re

import tqdm


class ParseDateTimeAction(argparse.Action):
    DATE_TIME_FORMAT = '%Y-%m-%d %H:%M:%S'

    def __call__(self, parser: argparse.ArgumentParser, namespace: argparse.Namespace, values, option_string=None):
        date_time_value = datetime.datetime.strptime(values, self.DATE_TIME_FORMAT)
        setattr(namespace, self.dest, date_time_value)


class ParseDateAction(argparse.Action):
    DATE_FORMAT = '%Y-%m-%d'

    def __call__(self, parser: argparse.ArgumentParser, namespace: argparse.Namespace, values, option_string=None):
        date_time_value = datetime.datetime.strptime(values, self.DATE_FORMAT)
        setattr(namespace, self.dest, date_time_value)


class ParseTimeDeltaAction(argparse.Action):
    TIME_DELTA_PATTERN = re.compile(r'^(?:(?P<days>\d+)\s+days?)?\s*(?:(?P<hours>\d+):(?P<minutes>\d+):(?P<seconds>\d+))?$')

    def __call__(self, parser: argparse.ArgumentParser, namespace: argparse.Namespace, values, option_string=None):
        match = self.TIME_DELTA_PATTERN.match(values)
        if not match:
            raise ValueError("Pattern '{0}' is not recognized as a timedelta".format(values))

        duration = datetime.timedelta()

        groups = match.groupdict()
        if 'days' in groups and groups['days']:
            duration += datetime.timedelta(days=int(groups['days']))

        if 'hours' in groups and groups['hours']:
            duration += datetime.timedelta(hours=int(groups['hours']))

        if 'minutes' in groups and groups['minutes']:
            duration += datetime.timedelta(minutes=int(groups['minutes']))

        if 'seconds' in groups and groups['seconds']:
            duration += datetime.timedelta(seconds=int(groups['seconds']))

        setattr(namespace, self.dest, duration)


def process_parallel_map(data, function):
    results = []
    with concurrent.futures.ProcessPoolExecutor() as executor:
        futures = [executor.submit(function, chunk) for chunk in data]
        for future in tqdm.tqdm(futures, leave=False):
            results.append(future.result())
    return results
