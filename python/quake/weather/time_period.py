import datetime


class TimePeriod:
    DATETIME_FORMAT: str = '%Y-%b-%d %H:%M:%S'

    def __init__(self, begin_time, end_time):
        self.__begin_time = begin_time
        self.__end_time = end_time

    def is_before(self, other):
        return self.end < other.begin

    def is_after(self, other):
        return self.begin >= other.end

    @property
    def begin(self):
        return self.__begin_time

    @property
    def end(self):
        return self.__end_time

    @property
    def length(self):
        return self.__end_time - self.__begin_time

    @staticmethod
    def from_json(json_object):
        begin_datetime = datetime.datetime.strptime(json_object['begin'], TimePeriod.DATETIME_FORMAT)
        end_datetime = datetime.datetime.strptime(json_object['end'], TimePeriod.DATETIME_FORMAT)
        return TimePeriod(begin_datetime, end_datetime)

    def to_json(self):
        return {'begin': self.__begin_time.strftime(self.DATETIME_FORMAT), 'end': self.__end_time.strftime(self.DATETIME_FORMAT)}
