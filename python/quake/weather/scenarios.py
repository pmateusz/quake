import collections
import datetime
import logging
import random
import warnings

import GPy
import numpy
import pandas
import scipy
import scipy.stats
import tqdm

import quake.city


class SampleSpace:
    NUM_OBSERVATIONS = 40

    class CachingSampler:
        def __init__(self, mean, variance, pool_size=100000):
            self.__mean = mean
            self.__variance = variance
            self.__pool_size = pool_size

            self.__pool = None
            self.__counter = 0

        def __call__(self):
            if self.__pool is None or self.__counter >= self.__pool.shape[1]:
                self.__pool = numpy.random.multivariate_normal(self.__mean,
                                                               numpy.diag(self.__variance),
                                                               size=self.__pool_size).transpose()
                self.__counter = 0
            sample = self.__pool[:, self.__counter]
            self.__counter += 1
            return sample

    def __init__(self, frames: list):
        samples = [self.frame_to_sample(frame) for frame in frames]
        self.__sample_matrix = numpy.column_stack(samples)
        self.__sum_of_squares = numpy.sum(self.__sample_matrix ** 2, axis=0)
        self.__mean = numpy.zeros(self.__sample_matrix.shape[0])

        self.__full_covariance = numpy.cov(self.__sample_matrix)
        self.__mean = numpy.mean(self.__sample_matrix, axis=1)
        self.__gaussian = scipy.stats.multivariate_normal(self.__mean, numpy.diag(self.__full_covariance), allow_singular=True)

    def heuristic_sample(self, num_samples):
        num_topics_per_sample = numpy.random.poisson(1.0, num_samples) + 1
        samples = []
        for num_topics in num_topics_per_sample:
            selected_topics = numpy.random.choice(self.__sample_matrix.shape[1], num_topics, replace=False)
            weights = numpy.random.normal(loc=1.0, scale=0.25, size=num_topics)
            # weights = numpy.ones(num_topics)
            # weights = numpy.random.dirichlet(numpy.full(num_topics, 0.05), 1)[0]
            sample = numpy.sum(numpy.dot(self.__sample_matrix[:, selected_topics], weights[0]), axis=1)
            samples.append(sample)
        return [self.sample_to_frame(sample) for sample in samples]

    def gaussian_sample(self, num_samples):
        samples = self.__gaussian.rvs(num_samples)
        return [self.sample_to_frame(sample) for sample in samples]

    def monte_carlo_markov_chain_sample(self, num_samples, burn_in_period=5000, independent_sample_step=1000):
        generator = random.Random(0)
        sampler = SampleSpace.CachingSampler(self.__mean, self.__full_covariance)
        accept_counter = 0
        reject_counter = 0

        iterations = burn_in_period + independent_sample_step * num_samples
        samples = []
        current_sample = self.__sample_matrix[:, generator.randint(0, self.__sample_matrix.shape[1])]
        current_sample_probability = self.heuristic_probability(current_sample)
        with tqdm.tqdm(range(iterations), leave=False) as iterator:
            for _ in iterator:
                raw_sample = sampler()
                candidate_sample = current_sample + raw_sample
                candidate_sample_probability = self.heuristic_probability(candidate_sample)
                probability_ratio = candidate_sample_probability / current_sample_probability

                accept_threshold = min(1.0, probability_ratio)
                if accept_threshold < 1.0:
                    # consider rejection
                    test_result = generator.uniform(0.0, 1.0)
                    if test_result > accept_threshold:
                        # rejection - continue using the same sample
                        reject_counter += 1
                        samples.append(current_sample)
                        continue
                # must accept
                accept_counter += 1
                current_sample = candidate_sample
                current_sample_probability = candidate_sample_probability
                samples.append(current_sample)

        logging.info('Metropolis accept vs. reject ration: (%d, %d)', accept_counter, reject_counter)

        shortlisted_samples = samples[burn_in_period:]
        shortlisted_samples = shortlisted_samples[::independent_sample_step]

        assert len(shortlisted_samples) == num_samples

        return [self.sample_to_frame(sample) for sample in shortlisted_samples]

    def heuristic_probability(self, sample: numpy.array) -> float:
        max_value = numpy.max(sample)
        if max_value > 100.0:
            return 0.0

        min_value = numpy.min(sample)
        if min_value < -100.0:
            return 0.0

        return 1.0 / (self.distance(sample) + 1.0)

    def distance(self, sample: numpy.array):
        return numpy.sum(-2 * numpy.dot(sample, self.__sample_matrix)
                         + numpy.sum(sample ** 2, axis=0)
                         + self.__sum_of_squares)

    @property
    def samples(self):
        return self.__sample_matrix

    def __slow_distance(self, sample: numpy.array):
        distances = []
        for sample_index in range(self.__sample_matrix.shape[1]):
            distances.append(numpy.sum((self.__sample_matrix[:, sample_index] - sample) ** 2))
        return numpy.sum(distances)

    @staticmethod
    def frame_to_sample(frame: pandas.DataFrame) -> numpy.array:
        if not (frame.index.size == SampleSpace.NUM_OBSERVATIONS and frame.index.inferred_freq == '3H' and not frame.index.has_duplicates):
            raise ValueError('Frame may have missing values')

        sample = []
        for city in quake.city.ALL:
            sample.extend(frame[city].values)

        return numpy.array(sample)

    @staticmethod
    def sample_to_frame(sample: numpy.array) -> pandas.DataFrame:
        data = []
        for offset in range(SampleSpace.NUM_OBSERVATIONS):
            row = sample[offset::SampleSpace.NUM_OBSERVATIONS].flatten()

            assert len(row) == len(quake.city.ALL)

            data.append(row)

        return pandas.DataFrame(data=data, columns=quake.city.ALL)


MeanVarianceResult = collections.namedtuple('MeanVarianceResult',
                                            ['confidence_interval',
                                             'mean', 'mean_lower', 'mean_upper',
                                             'variance', 'variance_lower', 'variance_upper'])


class PastErrorsScenarioGenerator:
    def __init__(self, weather_cache, forecast_frame):
        self.__weather_cache = weather_cache
        self.__forecast_frame = forecast_frame
        self.__sample_space = None
        self.__weather_samples = None

    def optimize(self):
        forecast_time_points = self.__weather_cache.get_forecast_time_points()
        error_frames = self.__weather_cache.get_error_frames(forecast_time_points)
        filter_frames = [error_frame for error_frame in error_frames if len(error_frame) == SampleSpace.NUM_OBSERVATIONS]
        self.__sample_space = SampleSpace(filter_frames)

        forecast_sample = self.__sample_space.frame_to_sample(self.__forecast_frame)

        weather_samples = []
        for sample_index in range(self.__sample_space.samples.shape[1]):
            weather_sample = self.__sample_space.samples[:, sample_index].T + forecast_sample
            weather_samples.append(weather_sample)
        self.__weather_samples = numpy.column_stack(weather_samples)

    def samples(self, size):
        if size > self.num_samples:
            warnings.warn('Too many samples requested. The generator will generate {0} samples out of {1} samples requested'
                          .format(self.num_samples, size))

        if size < self.num_samples:
            samples_selected = numpy.random.choice(self.num_samples, size, replace=False)
        else:
            samples_selected = numpy.arange(0, self.num_samples)

        samples = self.__weather_samples[:, samples_selected]

        frames = []
        for sample_index in range(samples.shape[1]):
            sample = numpy.copy(samples[:, sample_index])

            sample[sample > 100.0] = 100.0
            sample[sample < 0.0] = 0.0

            frame = self.__sample_space.sample_to_frame(sample)
            frame.set_index(self.__forecast_frame.index, inplace=True)
            frames.append(frame)
        return frames

    def bootstrap_mean_variance(self, iterations, batch_size=100, confidence=0.8) -> MeanVarianceResult:
        if confidence > 1.0 or confidence < 0.0:
            raise ValueError('confidence')

        mean = []
        variance = []

        for _ in tqdm.tqdm(range(iterations), leave=False):
            sample_indices = numpy.random.choice(self.num_samples, batch_size, replace=True)
            batch = self.__weather_samples[:, sample_indices]
            sample_mean = numpy.mean(batch, axis=1)
            mean.append(sample_mean)

            sample_variance = numpy.var(batch, axis=1)
            variance.append(sample_variance)

        mean = numpy.column_stack(mean)
        population_mean = numpy.mean(mean, axis=1)[:, numpy.newaxis]

        variance = numpy.column_stack(variance)
        population_variance = numpy.mean(variance, axis=1)[:, numpy.newaxis]

        confidence_fraction = (1.0 - confidence) / 2.0
        lower_conf_pos = int(numpy.floor(confidence_fraction * iterations))
        upper_conf_pos = int(numpy.ceil((1.0 - confidence_fraction) * iterations))
        upper_conf_pos = min(upper_conf_pos, iterations - 1)

        def get_confidence_intervals(sample_statistics, population_value):
            diff = numpy.sort(sample_statistics - population_value, axis=1)
            lower_conf = population_value + diff[:, lower_conf_pos][:, numpy.newaxis]
            upper_conf = population_value + diff[:, upper_conf_pos][:, numpy.newaxis]

            assert numpy.all(lower_conf <= population_value) and numpy.all(population_value <= upper_conf)

            return lower_conf, upper_conf

        mean_lower_ci, mean_upper_ci = get_confidence_intervals(mean, population_mean)
        variance_lower_ci, variance_upper_ci = get_confidence_intervals(variance, population_variance)

        trimmed_population_mean = numpy.copy(population_mean)
        trimmed_population_mean[trimmed_population_mean > 100.0] = 100.0
        trimmed_population_mean[trimmed_population_mean < 0.0] = 0.0

        mean_frame = self.__sample_space.sample_to_frame(trimmed_population_mean)
        mean_frame.set_index(self.__forecast_frame.index, inplace=True)

        mean_lower_ci_frame = self.__sample_space.sample_to_frame(mean_lower_ci)
        mean_lower_ci_frame.set_index(self.__forecast_frame.index, inplace=True)

        mean_upper_ci_frame = self.__sample_space.sample_to_frame(mean_upper_ci)
        mean_upper_ci_frame.set_index(self.__forecast_frame.index, inplace=True)

        variance_frame = self.__sample_space.sample_to_frame(population_variance)
        variance_frame.set_index(self.__forecast_frame.index, inplace=True)

        variance_lower_ci_frame = self.__sample_space.sample_to_frame(variance_lower_ci)
        variance_lower_ci_frame.set_index(self.__forecast_frame.index, inplace=True)

        variance_upper_ci_frame = self.__sample_space.sample_to_frame(variance_upper_ci)
        variance_upper_ci_frame.set_index(self.__forecast_frame.index, inplace=True)

        return MeanVarianceResult(confidence,
                                  mean_frame, mean_lower_ci_frame, mean_upper_ci_frame,
                                  variance_frame, variance_lower_ci_frame, variance_upper_ci_frame)

    @property
    def num_samples(self):
        return self.__weather_samples.shape[1]


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
            prediction = self.__forecast_frame[city].values + numpy.random.multivariate_normal(numpy.zeros(city_covariance.shape[0]),
                                                                                               city_covariance)
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
