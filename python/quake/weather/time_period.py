import datetime


class TimePeriod:
    DATETIME_FORMAT: str = '%Y-%b-%d %H:%M:%S'

    def __init__(self, begin_time: datetime.datetime, end_time: datetime.datetime):
        self.__begin_time = begin_time
        self.__end_time = end_time

    def is_before(self, other: 'TimePeriod') -> bool:
        return self.end < other.begin

    def is_after(self, other: 'TimePeriod') -> bool:
        return self.begin >= other.end

    def intersect(self, other_period: 'TimePeriod') -> 'TimePeriod':
        intersect_begin = max(self.begin, other_period.begin)
        intersect_end = min(self.end, other_period.end)
        return TimePeriod(intersect_begin, intersect_end)

    def contains(self, other_period: 'TimePeriod') -> bool:
        return self.begin <= other_period.begin and other_period.end <= self.end

    def __str__(self):
        return '[{0}/{1}]'.format(self.__begin_time.strftime(self.DATETIME_FORMAT), self.__end_time.strftime(self.DATETIME_FORMAT))

    def __eq__(self, other):
        if not isinstance(other, TimePeriod):
            return False

        return self.__begin_time == other.begin and self.__end_time == other.end

    def __hash__(self):
        return hash((self.__begin_time, self.__end_time))

    def __bool__(self) -> bool:
        return self.__begin_time < self.__end_time

    @property
    def begin(self) -> datetime.datetime:
        return self.__begin_time

    @property
    def end(self) -> datetime.datetime:
        return self.__end_time

    @property
    def length(self) -> datetime.timedelta:
        return self.__end_time - self.__begin_time

    @property
    def empty(self) -> bool:
        return self.__begin_time == self.__end_time

    @staticmethod
    def from_json(json_object) -> 'TimePeriod':
        begin_datetime = datetime.datetime.strptime(json_object['begin'], TimePeriod.DATETIME_FORMAT)
        end_datetime = datetime.datetime.strptime(json_object['end'], TimePeriod.DATETIME_FORMAT)
        return TimePeriod(begin_datetime, end_datetime)

    def to_json(self) -> dict:
        return {'begin': self.__begin_time.strftime(self.DATETIME_FORMAT), 'end': self.__end_time.strftime(self.DATETIME_FORMAT)}
