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

#include <numeric>

#include "forecast.h"
#include "scenario_pool.h"

quake::ScenarioPool::ScenarioPool()
        : problem_{nullptr} {}

quake::ScenarioPool::ScenarioPool(const ScenarioPool &other)
        : problem_{other.problem_},
          key_rate_index_{other.key_rate_index_} {}

quake::ScenarioPool::ScenarioPool(ScenarioPool &&other) noexcept
        : problem_{other.problem_},
          key_rate_index_{std::move(other.key_rate_index_)} {}

quake::ScenarioPool &quake::ScenarioPool::operator=(const ScenarioPool &other) {
    problem_ = other.problem_;
    key_rate_index_ = other.key_rate_index_;
    return *this;
}

quake::ScenarioPool &quake::ScenarioPool::operator=(ScenarioPool &&other) noexcept {
    problem_ = other.problem_;
    key_rate_index_ = std::move(other.key_rate_index_);
    return *this;
}

quake::ScenarioPool::ScenarioPool(const std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period>> &intervals,
                                  const std::vector<Forecast> &forecasts,
                                  quake::ExtendedProblem const *problem)
        : problem_{problem},
          key_rate_index_{} {

    const auto num_forecasts = forecasts.size();
    std::unordered_map<GroundStation, std::unordered_map<boost::posix_time::time_period, std::vector<double>>> key_rate_index{};
    for (const auto &station_element : intervals) {
        const auto &station = station_element.first;
        if (station == GroundStation::None) {
            continue;
        }

        std::unordered_map<boost::posix_time::time_period, std::vector<double>> station_index{};
        for (const auto &period : station_element.second) {
            std::vector<double> key_rate;
            key_rate.reserve(num_forecasts);
            for (const auto &forecast : forecasts) {
                key_rate.emplace_back(problem_->KeyRate(station, period, forecast));
            }

            const auto insert_pair = station_index.emplace(period, std::move(key_rate));
            CHECK(insert_pair.second);
        }
        const auto insert_pair = key_rate_index.emplace(station_element.first, std::move(station_index));
        CHECK(insert_pair.second);
    }

    key_rate_index_ = std::move(key_rate_index);
}

double quake::ScenarioPool::KeyRate(std::size_t scenario, const GroundStation &station, const boost::posix_time::time_period &interval) const {
    return key_rate_index_.at(station).at(interval).at(scenario);
}

double quake::ScenarioPool::KeyRate(const quake::GroundStation &station, const boost::posix_time::time_period &interval) const {
    const auto &key_rate_period = key_rate_index_.at(station).at(interval);
    CHECK(!key_rate_period.empty());

    double total_key_rate = std::accumulate(std::cbegin(key_rate_period), std::cend(key_rate_period), 0.0);
    return total_key_rate / key_rate_period.size();
}
