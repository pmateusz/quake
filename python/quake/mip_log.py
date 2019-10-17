import datetime
import re
import typing


class GurobiLog:
    __MIP_HEADER_PATTERN = re.compile('^\s*Expl\s+Unexpl\s+|\s+Obj\s+Depth\s+IntInf\s+|\s+Incumbent\s+BestBd\s+Gap\s+|\s+It/Node\s+Time\s*$')
    __MIP_LINE_PATTERN = re.compile('^(?P<solution_flag>[\w\*]?)\s*'
                                    '(?P<explored_nodes>\d+)\s+'
                                    '(?P<nodes_to_explore>\d+)\s+'
                                    '(?P<node_relaxation>[\w\.\+]*)\s+'
                                    '(?P<node_depth>\d*)\s+'
                                    '(?P<fractional_variables>\w*)\s+'
                                    '(?P<incumbent>[\w\.\-\+]*)\s+'
                                    '(?P<lower_bound>[\w\.\-\+]*)\s+'
                                    '(?P<gap>[\w\.\%\-]*)\s+'
                                    '(?P<simplex_it_per_node>[\d\.\-]*)\s+'
                                    '(?P<elapsed_time>\d+)s$')
    __SUMMARY_PATTERN = re.compile('^Best\sobjective\s(?P<objective>[e\d\.\+]+),\s'
                                   'best\sbound\s(?P<bound>[e\d\.\+]+),\s'
                                   'gap\s(?P<gap>[e\d\.\+]+)\%$')

    class MipProgressMessage:
        def __init__(self, has_solution, explored_nodes, best_cost, lower_bound, elapsed_time):
            self.__has_solution = has_solution
            self.__explored_nodes = explored_nodes
            self.__best_cost = best_cost
            self.__lower_bound = lower_bound
            self.__elapsed_time = elapsed_time

        @property
        def has_solution(self):
            return self.__has_solution

        @property
        def explored_nodes(self):
            return self.__explored_nodes

        @property
        def best_cost(self):
            return self.__best_cost

        @property
        def lower_bound(self):
            return self.__lower_bound

        @property
        def elapsed_time(self):
            return self.__elapsed_time

    def __init__(self, best_objective: float, best_bound: float, events: typing.List[MipProgressMessage]):
        self.__best_objective = best_objective
        self.__best_bound = best_bound
        self.__events = events

    @staticmethod
    def read_from_file(path) -> 'GurobiLog':
        events = []
        best_objective = float('inf')
        best_bound = float('-inf')
        with open(path, 'r') as fp:
            lines = fp.readlines()
            lines_it = iter(lines)
            for line in lines_it:
                if re.match(GurobiLog.__MIP_HEADER_PATTERN, line):
                    break
            next(lines_it, None)  # read the empty line
            for line in lines_it:
                line_match = re.match(GurobiLog.__MIP_LINE_PATTERN, line)
                if not line_match:
                    break

                raw_solution_flag = line_match.group('solution_flag')
                raw_incumbent = line_match.group('incumbent')
                raw_lower_bound = line_match.group('lower_bound')
                raw_elapsed_time = line_match.group('elapsed_time')
                raw_explored_nodes = line_match.group('explored_nodes')

                has_solution = raw_solution_flag == 'H' or raw_solution_flag == '*'
                incumbent = float(raw_incumbent) if raw_incumbent and raw_incumbent != '-' else float('inf')
                lower_bound = float(raw_lower_bound) if raw_lower_bound else float('-inf')
                elapsed_time = datetime.timedelta(seconds=int(raw_elapsed_time)) if raw_elapsed_time else datetime.timedelta()
                explored_nodes = int(raw_explored_nodes)
                events.append(GurobiLog.MipProgressMessage(has_solution, explored_nodes, incumbent, lower_bound, elapsed_time))
            next(lines_it, None)
            for line in lines_it:
                line_match = re.match(GurobiLog.__SUMMARY_PATTERN, line)
                if line_match:
                    raw_objective = line_match.group('objective')
                    if raw_objective:
                        best_objective = float(raw_objective)
                    raw_bound = line_match.group('bound')
                    if raw_bound:
                        best_bound = float(raw_bound)
        return GurobiLog(best_objective, best_bound, events)

    def best_cost(self) -> float:
        return self.__best_objective

    def best_cost_time(self) -> datetime.timedelta:
        for event in reversed(self.__events):
            if event.has_solution:
                return event.elapsed_time
        return datetime.timedelta.max

    def root_relaxation(self) -> None:
        filtered_events = [event for event in self.__events if event.explored_nodes == 0]
        if filtered_events:
            event = filtered_events[-1]
            if event.lower_bound == 0.0:
                return float('inf')
            # this gap is correct only for maximization problems
            return (event.lower_bound - event.best_cost) / event.best_cost
        return float('inf')

    def best_bound(self) -> float:
        return self.__best_bound

    def computation_time(self) -> datetime.timedelta:
        if self.__events:
            return self.__events[-1].elapsed_time
        return datetime.timedelta.max
