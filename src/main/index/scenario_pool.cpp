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

#include "forecast.h"
#include "scenario_pool.h"

quake::ScenarioPool::ScenarioPool()
        : model_{nullptr},
          num_scenarios_{0} {}

quake::ScenarioPool::ScenarioPool(const quake::ScenarioPool &other)
        : model_{other.model_},
          transferred_keys_index_{other.transferred_keys_index_},
          num_scenarios_{other.num_scenarios_} {}

quake::ScenarioPool::ScenarioPool(quake::ScenarioPool &&other) noexcept
        : model_{other.model_},
          transferred_keys_index_{std::move(other.transferred_keys_index_)},
          num_scenarios_{other.num_scenarios_} {}

quake::ScenarioPool &quake::ScenarioPool::operator=(const quake::ScenarioPool &other) {
    model_ = other.model_;
    transferred_keys_index_ = other.transferred_keys_index_;
    num_scenarios_ = other.num_scenarios_;
    return *this;
}

quake::ScenarioPool &quake::ScenarioPool::operator=(quake::ScenarioPool &&other) noexcept {
    model_ = other.model_;
    transferred_keys_index_ = std::move(other.transferred_keys_index_);
    num_scenarios_ = other.num_scenarios_;
    return *this;
}

quake::ScenarioPool::ScenarioPool(const std::vector<std::vector<quake::VarInterval> > &intervals,
                                  const std::vector<quake::Forecast> &forecasts,
                                  quake::InferredModel const *model)
        : model_{model},
          transferred_keys_index_{},
          num_scenarios_{forecasts.size()} {
    const auto num_scenarios = forecasts.size();
    for (const auto &leaf_intervals : intervals) {
        for (const auto &interval : leaf_intervals) {
            if (interval.StationIndex == model_->DummyStationIndex()) { continue; }

            const auto station = model->Station(interval.StationIndex);
            const auto begin_time = model->Time(interval.Begin);
            const auto end_time = model->Time(interval.End);
            std::vector<int64> transferred_keys(forecasts.size(), 0);
            for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
                transferred_keys[scenario_index] = model_->WeatherAdjustedTransferredKeys(station,
                                                                                          begin_time,
                                                                                          end_time,
                                                                                          forecasts.at(scenario_index));
            }

            BaseInterval base_interval{interval.StationIndex, interval.Begin, interval.End};
            const auto insert_pair = transferred_keys_index_.emplace(std::make_pair(base_interval, transferred_keys));
            CHECK(insert_pair.second);
        }
    }
}

int64 quake::ScenarioPool::KeysTransferred(std::size_t scenario, const quake::BaseInterval &interval) const {
    const auto find_it = transferred_keys_index_.find(interval);
    if (find_it == std::cend(transferred_keys_index_)) {
        LOG(FATAL) << "Interval not found";
    }
    return find_it->second.at(scenario);
}

int64 quake::ScenarioPool::MeanKeysTransferred(const quake::BaseInterval &interval) const {
    const auto find_it = transferred_keys_index_.find(interval);
    if (find_it == std::cend(transferred_keys_index_)) {
        LOG(FATAL) << "Interval not found";
    }
    CHECK(!find_it->second.empty());
    const auto total_keys_transferred
            = std::accumulate(std::cbegin(find_it->second), std::cend(find_it->second), static_cast<int64>(0));
    return total_keys_transferred / find_it->second.size();
}
