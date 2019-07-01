//
// Copyright 2018 Mateusz Polnik
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <random>

#include <glog/logging.h>

#include <boost/config.hpp>
#include <boost/math/distributions.hpp>

#include "forecast.h"
#include "gaussian_forecast_generator.h"

quake::GaussianForecastGenerator::GaussianForecastGenerator(double scaling_factor) :
        scaling_factor_{scaling_factor} {}

std::vector<quake::Forecast> quake::GaussianForecastGenerator::Generate(const Forecast &forecast,
                                                                        std::size_t scenarios) const {
    std::vector<Forecast> result;

    LOG(WARNING) << "Using the same seed sequence";
    std::seed_seq seed{0};
//    std::random_device random_device{};
//    std::seed_seq seed{random_device()};

    std::mt19937 generator{seed};
    std::normal_distribution<double> random_distribution;

    for (auto scenario_number = 0; scenario_number < scenarios; ++scenario_number) {
        std::unordered_map<GroundStation, Forecast::Series> perturbed_index;
        for (const auto &station_series_pair : forecast.Index()) {
            std::vector<Forecast::SeriesValueType> raw_series;

            for (const auto original_value : station_series_pair.second.Values()) {
                // values at the end of interval cause numerical problems
                double original_value_to_use = original_value;
                if (original_value == 0.0) {
                    original_value_to_use = 1;
                } else if (original_value == 100.0) {
                    original_value_to_use = 99.0;
                }

                const auto error_term = random_distribution(generator);
                const auto perturbed_value = LogitPerturbation(original_value_to_use, error_term);

                raw_series.push_back(static_cast<Forecast::SeriesValueType>(perturbed_value));
            }

            Forecast::Series series(station_series_pair.second.UpdateFrequency(),
                                    station_series_pair.second.Period(),
                                    std::move(raw_series));
            perturbed_index.insert({station_series_pair.first, std::move(series)});
        }

        result.emplace_back(std::move(perturbed_index));
    }

    return result;
}

// Perturbation operations implemented following the StackOverflow thread
// https://stats.stackexchange.com/questions/144410/how-to-add-noise-to-a-random-variable-whose-range-is-the-unit-interval
inline double logit(double value) {
    return log(value / (1.0 - value));
}

inline double delogit(double value) {
    return 1.0 / (1.0 + exp(-value));
}

double quake::GaussianForecastGenerator::LogitPerturbation(double initial_value, double error_term) const {
    double scaled_original_value = static_cast<double>(initial_value) / 100.0;
    CHECK_LE(scaled_original_value, 1.0);

    const auto original_projection = logit(scaled_original_value);
    const auto perturbed_projection = original_projection + error_term * scaling_factor_;
    const auto scaled_perturbed_value = delogit(perturbed_projection);
    const auto perturbed_value = std::round(scaled_perturbed_value * 100.0);

    CHECK_LE(perturbed_value, 100.0);
    CHECK_GE(perturbed_value, 0.0);

    return perturbed_value;
}

double quake::GaussianForecastGenerator::NormalPerturbation(double initial_value, double error_term) const {
    static const boost::math::normal_distribution<double> math_distribution;

    double scaled_original_value = static_cast<double>(initial_value) / 100.0;
    CHECK_LE(scaled_original_value, 1.0);

    const auto original_projection = boost::math::quantile(math_distribution, scaled_original_value);
    const auto perturbed_projection = original_projection + error_term * scaling_factor_;
    const auto scaled_perturbed_value = boost::math::cdf(math_distribution, perturbed_projection);
    const auto perturbed_value = std::round(scaled_perturbed_value * 100.0);

    CHECK_LE(perturbed_value, 100.0);
    CHECK_GE(perturbed_value, 0.0);

    return perturbed_value;
}
