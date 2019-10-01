import collections
import datetime
import json
import operator
import typing
import os

import pandas
import numpy

import quake.util
import quake.city
import quake.weather.metadata
import quake.weather.problem
import quake.weather.time_period


class Observation:

    def __init__(self, station: quake.city.City, period: quake.weather.time_period.TimePeriod):
        self.__station = station
        self.__period = period

    @property
    def period(self) -> quake.weather.time_period.TimePeriod:
        return self.__period

    @property
    def station(self) -> quake.city.City:
        return self.__station


class Solution:

    def __init__(self, json_object: typing.Dict[str, typing.Any]):
        self.__json_object = json_object

    def get_observations(self, station: quake.city.City) -> typing.List[quake.weather.time_period.TimePeriod]:
        return [observation.period for observation in self.observations if observation.station == station]

    def get_final_buffer(self, station: quake.city.City) -> float:
        for city_name, final_buffer in self.__json_object['final_buffers']:
            if quake.city.City.from_name(city_name) == station:
                return final_buffer
        return 0.0

    @property
    def observations(self) -> typing.List[Observation]:
        all_observations = [Observation(quake.city.from_name(station_name), quake.weather.time_period.TimePeriod.from_json(json_time_period))
                            for station_name, json_time_periods in self.__json_object['observations']
                            for json_time_period in json_time_periods]

        all_observations.sort(key=lambda observation: observation.period.begin)

        return all_observations

    @property
    def gap(self) -> float:
        return self.__metadata.get(quake.weather.metadata.GAP, None)

    @property
    def gap_limit(self) -> float:
        return self.__metadata.get(quake.weather.metadata.GAP_LIMIT, None)

    @property
    def interval_step(self) -> datetime.timedelta:
        return self.__metadata.get(quake.weather.metadata.INTERVAL_STEP, None)

    @property
    def observation_period(self) -> quake.weather.time_period.TimePeriod:
        return self.__metadata[quake.weather.metadata.OBSERVATION_PERIOD]

    @property
    def scenario_generator(self) -> str:
        return self.__metadata[quake.weather.metadata.SCENARIO_GENERATOR]

    @property
    def scenarios_number(self) -> int:
        return self.__metadata.get(quake.weather.metadata.SCENARIOS_NUMBER, None)

    @property
    def solution_type(self) -> str:
        return self.__metadata.get(quake.weather.metadata.SOLUTION_TYPE, None)

    @property
    def solution_method(self) -> str:
        return self.__metadata.get(quake.weather.metadata.SOLUTION_METHOD, None)

    @property
    def stations(self) -> typing.List[quake.city.City]:
        stations = [quake.city.City.from_name(station_name) for station_name, observations in self.__json_object['observations']]
        stations.sort(key=operator.attrgetter('latitude'), reverse=True)
        return stations

    @property
    def time_limit(self) -> datetime.timedelta:
        return self.__metadata.get(quake.weather.metadata.TIME_LIMIT, None)

    @property
    def target_traffic_index(self) -> float:
        return self.__metadata.get(quake.weather.metadata.TARGET_TRAFFIC_INDEX, None)

    @property
    def epsilon(self) -> float:
        return self.__metadata.get(quake.weather.metadata.EPSILON, None)

    @property
    def __metadata(self) -> typing.Dict[str, typing.Any]:
        return quake.weather.metadata.from_json(self.__json_object['metadata'])

    @staticmethod
    def read_json(file_path: str) -> 'Solution':
        with open(file_path, 'r') as input_stream:
            json_object = json.load(input_stream)
            return Solution(json_object)


class SolutionBundle:
    def __init__(self, problems: typing.List[quake.weather.problem.Problem], solutions: typing.List[Solution]):
        problem_by_date = {problem.observation_period.begin.date(): problem for problem in problems}
        solution_by_date = {solution.observation_period.begin.date(): solution for solution in solutions}

        dates = list(problem_by_date.keys())
        dates.extend(solution_by_date.keys())
        dates = list(set(dates))
        dates.sort()

        self.__problem_by_date = collections.OrderedDict()
        self.__solution_by_date = collections.OrderedDict()

        for date in dates:
            self.__problem_by_date[date] = problem_by_date[date]
            self.__solution_by_date[date] = solution_by_date[date]

    def get_transfer_share(self, station: quake.city.City) -> float:
        return self.__sample_problem.get_transfer_share(station)

    @property
    def stations(self) -> typing.List[quake.city.City]:
        return self.__sample_problem.stations

    def to_frame(self) -> pandas.DataFrame:
        data = []
        for date in self.__problem_by_date:
            problem = self.__problem_by_date[date]
            scenario = problem.get_scenario('real')
            solution = self.__solution_by_date[date]
            for station in problem.stations:
                local_keys_transferred = 0
                for observation in solution.get_observations(station):
                    local_keys_transferred += problem.get_transferred_keys(station, observation, scenario)
                local_final_buffer = solution.get_final_buffer(station)
                local_initial_buffer = local_final_buffer - local_keys_transferred
                assert local_initial_buffer >= 0

                data.append({'station': station, 'date': date, 'initial_buffer': local_initial_buffer, 'keys_transferred': local_keys_transferred})
        frame = pandas.DataFrame(data=data)
        frame.set_index(['station', 'date'], inplace=True)
        frame.sort_index(level=1, inplace=True)
        return frame

    def to_local_service_level_frame(self) -> pandas.DataFrame:
        bundle_frame = self.to_frame()
        stations = bundle_frame.index.get_level_values(0).unique()

        data = []
        for station in stations:
            keys_transferred = bundle_frame.loc[station]['keys_transferred'].values

            weekly_consumption = 0
            while True:
                local_deltas = keys_transferred - weekly_consumption
                local_balance = numpy.cumsum(local_deltas)
                negative_balance_weeks = local_balance[local_balance < 0]

                data.append({'station': station,
                             'weekly_consumption': weekly_consumption,
                             'negative_balance_weeks': negative_balance_weeks.size,
                             'total_weeks': local_deltas.size})

                if len(negative_balance_weeks) == len(keys_transferred):
                    break

                weekly_consumption += 1.0

        master_frame = pandas.DataFrame(data=data)
        master_frame.sort_values(by=['station', 'weekly_consumption'], inplace=True)
        master_frame.set_index(['station', 'weekly_consumption'], inplace=True)
        return master_frame

    def to_global_service_level_frame(self) -> pandas.DataFrame:
        bundle_frame = self.to_frame()
        stations = bundle_frame.index.get_level_values(0).unique()

        data = []
        traffic_index = 0.0
        while True:
            total_negative_balance_pos = set()
            total_pos = 0
            for station in stations:
                transfer_share = self.get_transfer_share(station)
                keys_transferred = bundle_frame.loc[station]['keys_transferred'].values
                total_pos = len(keys_transferred)
                delta = keys_transferred - traffic_index * transfer_share

                local_balance = numpy.cumsum(delta)
                positive_balance = local_balance[local_balance > 0]
                negative_balance_pos = numpy.argwhere(local_balance < 0).flatten()
                if negative_balance_pos.size == 0:
                    continue

                total_negative_balance_pos = total_negative_balance_pos.union(set(negative_balance_pos.tolist()))
                if positive_balance.size == 0:
                    break

            data.append({'traffic_index': traffic_index,
                         'negative_balance_weeks': len(total_negative_balance_pos),
                         'total_weeks': total_pos})

            if len(total_negative_balance_pos) == total_pos:
                break

            traffic_index += 10.0
        return pandas.DataFrame(data=data)

    @staticmethod
    def read_from_dir(problem_directory_path: str, solution_directory_path: str) -> 'SolutionBundle':
        problem_bundle = quake.weather.problem.ProblemBundle.read_from_dir(problem_directory_path)

        solution_files = []
        for file_name in os.listdir(solution_directory_path):
            if file_name.startswith('solution_'):
                solution_path = os.path.join(solution_directory_path, file_name)
                solution_files.append(solution_path)

        solutions = quake.util.process_parallel_map(solution_files, quake.weather.solution.Solution.read_json)
        solutions.sort(key=lambda solution: solution.observation_period.begin)
        return SolutionBundle(problem_bundle.problems, solutions)

    @property
    def __sample_problem(self) -> quake.weather.problem.Problem:
        first_date = list(self.__problem_by_date)[0]
        first_problem = self.__problem_by_date[first_date]
        return first_problem
