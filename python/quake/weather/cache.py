import collections
import concurrent.futures
import datetime
import json
import logging
import os
import time
import warnings

import numpy
import pandas
import tqdm

import quake.city


def load_data_frame(file_path):
    with open(file_path, 'r') as file_stream:
        master_record = json.load(file_stream)
        data = []
        for leaf_record in master_record:
            city = quake.city.from_key(leaf_record['city']['id'])
            count = int(leaf_record['cnt'])
            status_code = int(leaf_record['cod'])
            assert status_code == 200

            local_records = 0
            for record in leaf_record['list']:
                time = datetime.datetime.strptime(record['dt_txt'], '%Y-%m-%d %H:%M:%S')
                cloud_cover = record['clouds']['all']
                description = record['weather'][0]['description']
                data.append((city, time, cloud_cover, description))
                local_records += 1
            assert local_records == count

        data_frame = pandas.DataFrame(columns=['City', 'DateTime', 'CloudCover', 'Description'], data=data)
        min_time = data_frame['DateTime'].min()
        data_frame['Delay'] = data_frame['DateTime'] - min_time
        return data_frame


class WeatherCache:
    HDF_FILE_TABLE = 'a'
    HDF_FILE_MODE = 'w'
    HDF_FORMAT = 'fixed'
    PERCENTAGE_MISSING_VALUES_THRESHOLD = 0.10
    FORECAST_CACHE_FILE = '/home/pmateusz/dev/quake/current_review/forecast.hdf'
    OBSERVATION_CACHE_FILE = '/home/pmateusz/dev/quake/current_review/observation.hdf'
    ZERO_TIME_DELTA = datetime.timedelta()

    def __init__(self, load_historical_observations=False):
        self.__forecast_frame = None
        self.__observation_frame = None
        self.__forecast_length = datetime.timedelta(days=4, hours=21)
        self.__forecast_frequency = datetime.timedelta(hours=3)
        self.__load_historical_observations = load_historical_observations

    def rebuild(self):

        def load_observation_frame(file_path):
            _, ext = os.path.splitext(file_path)
            if ext == '.csv':
                data_frame = pandas.read_csv(file_path)
            elif ext == '.json':
                data = []
                with open(file_path, 'r') as input_stream:
                    records = json.load(input_stream)
                    for record in records:
                        record_dict = {}
                        for master_key in record:
                            if isinstance(record[master_key], dict):
                                if master_key == 'main':
                                    for leaf_key in record[master_key]:
                                        record_dict[leaf_key] = record[master_key][leaf_key]
                                else:
                                    for leaf_key in record[master_key]:
                                        record_dict[master_key + '_' + leaf_key] = record[master_key][leaf_key]
                            elif isinstance(record[master_key], list):
                                if record[master_key]:
                                    for leaf_key in record[master_key][0]:
                                        record_dict[master_key + '_' + leaf_key] = record[master_key][0][leaf_key]
                            elif record[master_key] is None:
                                continue
                            else:
                                record_dict[master_key] = record[master_key]
                        data.append(record_dict)
                data_frame = pandas.DataFrame(data=data)
            else:
                raise ValueError("File format '{0}' is not supported".format(ext))

            def parse_datetime(text: str) -> datetime.datetime:
                local_date_time = datetime.datetime.strptime(text, '%Y-%m-%d %H:%M:%S %z %Z')

                assert local_date_time.timetz().tzname() == 'UTC'

                seconds = time.mktime(local_date_time.timetuple())
                utc_time = time.gmtime(seconds)
                utc_date_time = datetime.datetime(year=utc_time.tm_year, month=utc_time.tm_mon, day=utc_time.tm_mday,
                                                  hour=utc_time.tm_hour, minute=utc_time.tm_min, second=utc_time.tm_sec)
                return utc_date_time

            data_frame['DateTime'] = data_frame['dt_iso'].apply(parse_datetime)
            data_frame['City'] = data_frame['city_id'].apply(quake.city.from_key)
            data_frame.rename(columns={'clouds_all': 'CloudCover'}, inplace=True)
            return data_frame[['DateTime', 'City', 'CloudCover']].copy()

        observation_frame_main = load_observation_frame('/home/pmateusz/dev/quake/data/weather/ground_station_data_set_filled.csv')
        observation_frame_belfast = load_observation_frame('/home/pmateusz/dev/quake/data/weather/belfast.json')
        observation_frame = pandas.concat([observation_frame_main, observation_frame_belfast])
        observation_frame = observation_frame.infer_objects()
        observation_frame.set_index(keys=['DateTime', 'City'], inplace=True, drop=True)
        observation_frame.sort_index(0, inplace=True)

        self.__observation_frame = observation_frame

        resolved_root_directory = os.path.expanduser('~/OneDrive/dev/quake/data/forecasts/')
        file_paths = [os.path.abspath(os.path.join(resolved_root_directory, file_name))
                      for file_name in os.listdir(resolved_root_directory) if file_name.endswith('.json')]

        data_frames = []
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', '', tqdm.TqdmSynchronisationWarning)
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future_to_path = {executor.submit(load_data_frame, file_path): file_path for file_path in file_paths}
                for future in tqdm.tqdm(concurrent.futures.as_completed(future_to_path), total=len(future_to_path)):
                    file_path = future_to_path[future]
                    try:
                        data_frames.append(future.result())
                    except Exception as ex:
                        logging.exception('%s generated error %s', file_path, ex)
        forecast_frame = pandas.concat(data_frames)
        forecast_frame.drop_duplicates(inplace=True)
        forecast_frame = forecast_frame.infer_objects()
        forecast_frame.set_index(['DateTime', 'Delay', 'City'], inplace=True, drop=True)
        forecast_frame.sort_index(0, inplace=True)

        self.__forecast_frame = forecast_frame

    def load(self):
        self.__forecast_frame = pandas.read_hdf(self.FORECAST_CACHE_FILE, self.HDF_FILE_TABLE)

        if self.__load_historical_observations:
            self.__observation_frame = pandas.read_hdf(self.OBSERVATION_CACHE_FILE, self.HDF_FILE_TABLE)
        else:
            warnings.warn('Disabled loading of historical observation data frame for performance reasons')

    def save(self):
        self.__forecast_frame.to_hdf(self.FORECAST_CACHE_FILE, self.HDF_FILE_TABLE, mode=self.HDF_FILE_MODE, format=self.HDF_FORMAT)

        if self.__load_historical_observations:
            observation_frame = self.__observation_frame.copy()
            observation_frame.to_hdf(self.OBSERVATION_CACHE_FILE, self.HDF_FILE_TABLE, mode=self.HDF_FILE_MODE, format=self.HDF_FORMAT)

    def get_forecast_frame(self, start_time, duration):
        end_time = start_time + duration
        first_filter_frame = self.__forecast_frame[start_time:end_time]
        second_filter_frame = first_filter_frame[first_filter_frame.index.get_level_values('DateTime')
                                                 - first_filter_frame.index.get_level_values('Delay') == start_time]
        return self.__pivot_transform(second_filter_frame)

    def get_observation_frame(self, start_time, duration):
        end_time = start_time + duration
        data_frame = self.__forecast_frame.loc[pandas.IndexSlice[start_time:end_time, self.ZERO_TIME_DELTA], :]

        if data_frame.empty:
            data_frame = self.__observation_frame.loc[pandas.IndexSlice[start_time:end_time], :].copy()

        return self.__pivot_transform(data_frame)

    def get_closest_forecast_frame(self, start_time, duration):
        end_time = start_time + duration

        filter_frame = self.__forecast_frame[start_time:end_time].copy()

        if filter_frame.empty:
            warnings.warn('No forecast records found in requested period [{0}, {1}]'.format(start_time, end_time))
            return pandas.DataFrame(columns=self.__forecast_frame.columns)

        filter_frame['ForecastDateTime'] = filter_frame.index.get_level_values('DateTime') - filter_frame.index.get_level_values('Delay')
        filter_frame['ForecastOffsetDiff'] = filter_frame['ForecastDateTime'].apply(lambda value: abs((value - start_time).total_seconds()))

        min_forecast_date_time_diff = filter_frame['ForecastOffsetDiff'].min()
        forecast_date_time = filter_frame[filter_frame['ForecastOffsetDiff'] == min_forecast_date_time_diff]
        return self.__pivot_transform(forecast_date_time)

    def get_forecast_time_points(self):
        filter_frame = self.__forecast_frame.loc[pandas.IndexSlice[:, self.ZERO_TIME_DELTA, quake.city.LONDON], :]
        time_points = list(filter_frame.index.get_level_values('DateTime').unique().to_pydatetime())
        time_points.sort()

        def is_midnight_or_noon(time_point: datetime.datetime) -> bool:
            return time_point.hour % 12 == 0

        filter_time_points = [time_point for time_point in time_points if is_midnight_or_noon(time_point)]
        return filter_time_points

    def get_error_frames(self, time_points):
        result = []

        for time_point in tqdm.tqdm(time_points, desc='Creating Error Frames', leave=False):
            forecast_frame = self.get_forecast_frame(time_point, self.forecast_length)
            observation_frame = self.get_observation_frame(time_point, self.forecast_length)
            error_frame = forecast_frame - observation_frame
            error_frame['Delay'] = error_frame.index - error_frame.index.min()
            error_frame.dropna(inplace=True)
            if error_frame.empty:
                continue
            result.append(error_frame)
        return result

    def get_std_frame(self):
        time_points = self.get_forecast_time_points()
        diff_frames = self.get_error_frames(time_points)
        diff_frame = pandas.concat(diff_frames, ignore_index=True)
        del diff_frames

        delay_values = list(pandas.to_timedelta(value, unit='ns') for value in diff_frame['Delay'].unique())
        delay_values.sort()

        rows = []
        cities = [column for column in diff_frame.columns if isinstance(column, quake.city.City)]
        for delay_value in delay_values:
            filter_frame = diff_frame[diff_frame['Delay'] == delay_value].copy()
            row = {city: filter_frame[city].std() for city in cities}
            rows.append(row)
        return pandas.DataFrame(index=delay_values, data=rows)

    def get_forecast_error_variance(self, error_frames=None):
        if not error_frames:
            error_frames = self.get_error_frames(self.get_forecast_time_points())
        master_error_frame = pandas.concat(error_frames)
        master_error_frame['DateTime'] = master_error_frame.index
        master_error_frame.reset_index(drop=True, inplace=True)

        city_columns = [column for column in master_error_frame.columns if isinstance(column, quake.city.City)]
        melted_master_error_frame = pandas.melt(master_error_frame, value_vars=city_columns,
                                                var_name='City', value_name='Error', id_vars=['DateTime', 'Delay'])
        var_series = melted_master_error_frame.groupby(['City', 'Delay'])['Error'].var()
        return var_series

    def get_forecast_error_covariance(self, city, error_frames):
        rows = []
        time_steps = [datetime.timedelta(hours=value) for value in range(0,
                                                                         int(self.__forecast_length.total_seconds() / 3600) + 1,
                                                                         int(self.__forecast_frequency.total_seconds() / 3600))]
        for error_frame in error_frames:
            min_time = error_frame.index.min()
            max_time = error_frame.index.max()

            series = error_frame[city]
            series_index_set = set(series.index.to_pydatetime().tolist())
            current_time = min_time
            while current_time < max_time:
                row = []
                for time_step in time_steps:
                    time_index = current_time + time_step
                    if time_index in series_index_set:
                        row.append(series[time_index])
                    else:
                        row.append(numpy.NaN)
                rows.append(row)
                current_time = current_time + self.__forecast_frequency
        master_frame = pandas.DataFrame(data=rows, columns=time_steps)
        covariance = master_frame.cov()
        return covariance.values

    @staticmethod
    def fill_missing_values(frame):
        if len(frame) == 1:
            return frame

        freq = pandas.infer_freq(frame.index)
        if freq:
            if not frame.index.freq:
                frame.set_index(pandas.DatetimeIndex(frame.index.values, freq=freq), inplace=True)
            return frame

        counter = collections.Counter()
        index_it = iter(frame.index)
        prev_value = next(index_it, None)
        if prev_value:
            for current_value in index_it:
                time_distance = current_value - prev_value
                counter[time_distance] += 1
                prev_value = current_value

        inferred_feq = counter.most_common(1)[0][0]
        start_index = frame.index.min()
        end_index = frame.index.max()

        missing_values = []
        current_index = start_index
        while current_index < end_index:
            if current_index not in frame.index:
                missing_values.append(current_index)
            current_index += inferred_feq

        percentage_missing_values = float(len(missing_values)) / (frame.shape[0] + len(missing_values))
        if percentage_missing_values > WeatherCache.PERCENTAGE_MISSING_VALUES_THRESHOLD:
            warnings.warn('Missing values constitute {0:.2f}% of all values in the frame which exceeds {1}% threshold'
                          .format(percentage_missing_values * 100.0, WeatherCache.PERCENTAGE_MISSING_VALUES_THRESHOLD * 100.0))

        data = numpy.full((len(missing_values), len(frame.columns)), numpy.NaN)

        missing_values_frame = pandas.DataFrame(index=missing_values, data=data, columns=frame.columns)
        filled_frame = frame.append(missing_values_frame)
        filled_frame.sort_index(inplace=True)
        filled_frame.fillna(method='ffill', inplace=True)

        freq = pandas.infer_freq(filled_frame.index)
        if freq:
            filled_frame.set_index(pandas.DatetimeIndex(filled_frame.index.values, freq=freq), inplace=True)

        return filled_frame

    @staticmethod
    def __verify_period(frame, start_time, end_time):
        actual_start_time = frame['DateTime'].min()
        actual_end_time = frame['DateTime'].max()

        if actual_start_time > start_time or actual_end_time < end_time:
            warnings.warn('Available data [{0},{1}] does not cover requested period [{2}, {3}]'
                          .format(actual_start_time, actual_end_time, start_time, end_time))

    @staticmethod
    def __pivot_transform(frame):
        pivot_frame = pandas.pivot_table(frame, columns=['City'], index=['DateTime'], values=['CloudCover'])
        pivot_frame.columns = pivot_frame.columns.droplevel()
        pivot_frame.drop_duplicates(inplace=True)
        pivot_frame.sort_index(inplace=True)
        return pivot_frame

    @property
    def forecast_length(self):
        return self.__forecast_length

    @property
    def forecast_frame(self):
        return self.__forecast_frame

    @property
    def observation_frame(self):
        return self.__observation_frame
