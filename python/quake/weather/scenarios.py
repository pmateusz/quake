import datetime

import pandas
import numpy
import GPy
import tqdm

import quake.city


class CoregionalizationModel:

    def __init__(self, observation_frame, forecast_frame):
        assert observation_frame.empty or observation_frame.index.max() <= forecast_frame.index.min()
        assert not forecast_frame.empty

        self.__observation_frame = observation_frame
        self.__forecast_frame = forecast_frame
        self.__model = None
        self.__X = None
        self.__Y = None

    def optimize(self):
        # prepare input
        X, Y = [], []
        observation_start_time = self.__observation_frame.index.min() if not self.__observation_frame.empty else self.__forecast_frame.index.min()

        def elapsed_hours(date_time):
            return int((date_time - observation_start_time).total_seconds() / 3600)

        cities = self.__forecast_frame.columns
        for city_index, city in enumerate(cities):
            if not self.__observation_frame.empty:
                for time, cloud_cover in self.__observation_frame[city].items():
                    X.append([time, elapsed_hours(time), city_index])
                    Y.append([cloud_cover])

            for time, cloud_cover in self.__forecast_frame[city].items():
                X.append([time, elapsed_hours(time), city_index])
                Y.append([cloud_cover])

        self.__X = numpy.array(X)
        self.__Y = numpy.array(Y)
        del X, Y

        # plot output data
        matrix_rank = len(cities)
        kernel_1 = GPy.kern.RBF(1, lengthscale=6, ARD=True) \
                   + GPy.kern.Linear(1, active_dims=[0], ARD=True) \
                   + GPy.kern.White(1) \
                   + GPy.kern.Bias(1)
        kernel_2 = GPy.kern.Coregionalize(1, output_dim=len(cities), rank=matrix_rank)

        X_Gpy = self.__X[:, [1, 2]]
        self.__model = GPy.models.GPRegression(X_Gpy, self.__Y, kernel_1 ** kernel_2)
        self.__model.optimize(messages=True)

        mean, variance = self.__model.predict(X_Gpy, full_cov=True)
        quantiles = self.__model.predict_quantiles(X_Gpy)

        X_cities = numpy.array([cities[city_index] for city_index in self.__X[:, 2]])
        data = numpy.hstack((self.__X[:, [0]], X_cities.reshape(-1, 1), self.__Y.reshape(-1, 1), mean, quantiles[0], quantiles[1]))
        data_frame = pandas.DataFrame(data=data, columns=['DateTime', 'City', 'y', 'yhat', 'yhat_lower', 'yhat_upper'])
        data_frame = data_frame.infer_objects()
        return data_frame

    def samples(self, size):
        test_X = self.__X[self.__X[:, 0] >= self.__forecast_frame.index.min()]
        Y_posterior_samples = self.__model.posterior_samples_f(test_X[:, [1, 2]], full_cov=True, size=size)
        time_index = test_X[:, [0]]

        cities = self.__forecast_frame.columns
        X_cities = numpy.array([cities[city_index] for city_index in test_X[:, 2]])
        X_cities = X_cities.reshape(-1, 1)

        sample_frames = []
        for sample_index in range(size):
            data = numpy.hstack((time_index, X_cities, Y_posterior_samples[:, :, sample_index]))
            data_frame = pandas.DataFrame(data=data, columns=['DateTime', 'City', 'y'])
            data_frame = data_frame.infer_objects()
            sample_frames.append(data_frame)
        return sample_frames


class PastErrorsModel:
    DATE_TIME_FREQ = datetime.timedelta(hours=3)

    def __init__(self, weather_cache, forecast_frame):
        self.__weather_cache = weather_cache
        self.__forecast_frame = forecast_frame
        self.__error_frames = []

    def optimize(self):
        error_frames = self.__weather_cache.get_error_frames(self.__weather_cache.get_forecast_time_points())
        num_rows_min = self.__forecast_frame.shape[0]
        num_missing_rows_max = int(0.2 * num_rows_min)
        num_cities = len([column for column in self.__forecast_frame.columns if isinstance(column, quake.city.City)])

        # get error frames that do not have many missing entries and cover forecast's period fully
        valid_error_frames = []

        for error_frame in error_frames:
            assert not error_frame.empty

            first_date_time = error_frame.index[0] - error_frame.iloc[0]['Delay']
            last_date_time = error_frame.index.max()

            num_rows_after_filling = int((last_date_time - first_date_time).total_seconds() / self.DATE_TIME_FREQ.total_seconds()) + 1
            if num_rows_after_filling < num_rows_min:
                continue

            num_missing_rows = num_rows_min - error_frame.shape[0]
            if num_missing_rows > num_missing_rows_max:
                continue

            missing_entries = []
            current_date_time = first_date_time
            while current_date_time <= last_date_time:
                if current_date_time not in error_frame.index:
                    missing_entries.append(current_date_time)
                current_date_time += self.DATE_TIME_FREQ

            if not missing_entries:
                valid_error_frames.append(error_frame)
                continue

            # create frame with missing entries
            missing_rows = []
            for entry in missing_entries:
                row = [numpy.NaN] * num_cities
                row.append(entry - first_date_time)
                missing_rows.append(row)

            missing_values_frame = pandas.DataFrame(index=missing_entries, data=missing_rows, columns=error_frame.columns)

            # join frames
            filled_error_frame = error_frame.append(missing_values_frame)
            filled_error_frame.sort_index(inplace=True)
            filled_error_frame.fillna(value=0.0, inplace=True)
            valid_error_frames.append(filled_error_frame)

        self.__error_frames = valid_error_frames

    def samples(self, size):
        draw_with_replacement = size > len(self.__error_frames)
        samples_selected = numpy.random.choice(len(self.__error_frames), replace=draw_with_replacement, size=size)

        # column names for a sample frame are: DateTime, City and y
        sample_frames = []
        for sample_index in samples_selected:
            error_frame = self.__error_frames[sample_index]
            values = self.__forecast_frame.values + error_frame.values[:len(self.__forecast_frame),
                                                    :len(error_frame.columns) - 1]  # trim the last delay column
            rows = []
            for city_index, city in enumerate(self.__forecast_frame.columns):
                for row_index, date_time in enumerate(self.__forecast_frame.index):
                    rows.append([date_time, city, values[row_index, city_index]])
            sample_frame = pandas.DataFrame(columns=['DateTime', 'City', 'y'], data=rows)
            sample_frames.append(sample_frame)
        return sample_frames


class HeteroscedasticIndependentNoiseModel:

    def __init__(self, weather_cache, forecast_frame):
        self.__weather_cache = weather_cache
        self.__forecast_frame = forecast_frame
        self.__step_city_variance = None

    def optimize(self):
        error_frames = self.__weather_cache.get_error_frames(self.__weather_cache.get_forecast_time_points())
        self.__step_city_variance = self.__weather_cache.get_forecast_error_variance(error_frames)

    def samples(self, size):
        sample_frames = [self.__generate_sample_frame() for _ in range(size)]
        return sample_frames

    def __generate_sample_frame(self):
        num_rows = self.__forecast_frame.shape[0]

        rows = []
        for city in self.__forecast_frame.columns:
            V = self.__step_city_variance[city].values
            prediction = self.__forecast_frame[city].values + numpy.random.normal(size=num_rows) * numpy.sqrt(V[:num_rows])
            for time_delta, value in zip(self.__forecast_frame.index, prediction):
                rows.append([city, time_delta, value])
        sample_frame = pandas.DataFrame(data=rows, columns=['City', 'DateTime', 'y'])
        return sample_frame


class HeteroscedasticAutoCorrelatedNoiseModel:

    def __init__(self, weather_cache, forecast_frame):
        self.__weather_cache = weather_cache
        self.__forecast_frame = forecast_frame
        self.__city_covariance = None

    def optimize(self):
        error_frames = self.__weather_cache.get_error_frames(self.__weather_cache.get_forecast_time_points())

        num_forecast_rows = len(self.__forecast_frame)
        self.__city_covariance = dict()
        for city in tqdm.tqdm(self.__forecast_frame.columns, leave=False, desc='Building covariance matrices'):
            full_covariance_matrix = self.__weather_cache.get_forecast_error_covariance(city, error_frames)

            # limit covariance matrix to weather forecast
            covariance_matrix_to_use = full_covariance_matrix[0:num_forecast_rows, 0:num_forecast_rows]
            self.__city_covariance[city] = covariance_matrix_to_use

    def samples(self, size):
        sample_frames = [self.__generate_sample_frame() for _ in range(size)]
        return sample_frames

    def __generate_sample_frame(self):
        rows = []
        for city in self.__forecast_frame.columns:
            city_covariance = self.__city_covariance[city]
            prediction = self.__forecast_frame[city].values + numpy.random.multivariate_normal(numpy.zeros(city_covariance.shape[0]), city_covariance)
            for time_delta, value in zip(self.__forecast_frame.index, prediction):
                rows.append([city, time_delta, value])
        sample_frame = pandas.DataFrame(data=rows, columns=['City', 'DateTime', 'y'])
        return sample_frame


class ScenarioGeneratorFactory:
    PAST_ERRORS_MODEL_NAME = 'past_error_replication'
    INDEPENDENT_NOISE_MODEL_NAME = 'independent_error_simulation'
    CORRELATED_NOISE_MODEL_NAME = 'correlated_error_simulation'
    COREGIONALIZATION_MODEL_NAME = 'coregionalization'

    MODEL_NAMES = [PAST_ERRORS_MODEL_NAME, INDEPENDENT_NOISE_MODEL_NAME, CORRELATED_NOISE_MODEL_NAME, COREGIONALIZATION_MODEL_NAME]

    def create_model(self, model_name, weather_cache, forecast_frame):
        if model_name == self.PAST_ERRORS_MODEL_NAME:
            return PastErrorsModel(weather_cache, forecast_frame)
        elif model_name == self.INDEPENDENT_NOISE_MODEL_NAME:
            return HeteroscedasticIndependentNoiseModel(weather_cache, forecast_frame)
        elif model_name == self.CORRELATED_NOISE_MODEL_NAME:
            return HeteroscedasticAutoCorrelatedNoiseModel(weather_cache, forecast_frame)
        elif model_name == self.COREGIONALIZATION_MODEL_NAME:
            default_observation_length = datetime.timedelta(days=14)
            observation_frame \
                = weather_cache.get_observation_frame(forecast_frame.index.min() - default_observation_length, default_observation_length)
            return CoregionalizationModel(observation_frame, forecast_frame)
        assert False, 'Model name \"{0}\" is invalid' % model_name
