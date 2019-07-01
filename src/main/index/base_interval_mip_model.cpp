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

#include <unordered_set>

#include <glog/logging.h>

#include "base_interval_mip_model.h"
#include "fixed_discretisation_scheme.h"

quake::BaseIntervalMipModel::BaseIntervalMipModel(const quake::InferredModel *model,
                                                  boost::posix_time::time_duration time_step,
                                                  std::vector<quake::Forecast> forecasts)
        : BaseMipModel(model, std::move(time_step)),
          forecasts_{std::move(forecasts)} {}

void quake::BaseIntervalMipModel::Build(const boost::optional<Solution> &solution) {
    CHECK(intervals_.empty());

    // variables: create intervals
    FixedDiscretisationScheme scheme{*model_, time_step_};
    const auto intervals = scheme.Build();

    intervals_.resize(num_stations_);
    for (const auto &prototype : intervals.SwitchIntervals) {
        intervals_.at(dummy_station_).emplace_back(VarInterval::Create(mip_model_, prototype));
    }

    for (std::size_t station_index = 0; station_index < num_stations_; ++station_index) {
        if (station_index == dummy_station_) { continue; }

        for (const auto &prototype : intervals.TransferIntervals) {
            intervals_.at(station_index).emplace_back(
                    VarInterval::Create(mip_model_,
                                        {station_index,
                                         prototype.Begin,
                                         prototype.End}));
        }
    }

    scenario_pool_ = ScenarioPool(intervals_, forecasts_, model_);

    // constraint: at most one interval is active
    std::vector<GRBLinExpr> active_intervals(num_time_);
    for (auto station_index = 0; station_index < num_stations_; ++station_index) {
        for (const auto &interval : intervals_.at(station_index)) {
            const std::size_t end_to_use = std::min(num_time_, interval.End);
            for (auto time_index = interval.Begin; time_index < end_to_use; ++time_index) {
                active_intervals.at(time_index) += interval.Var;
            }
        }
    }

    for (auto time_index = 0; time_index < num_time_; ++time_index) {
        mip_model_.addConstr(active_intervals.at(time_index) <= 1.0);
    }

    // constraint: ensure a dummy interval is placed between intervals of different stations
    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        const auto num_intervals = intervals_.at(station_index).size();
        for (auto next_interval_index = 1; next_interval_index < num_intervals; ++next_interval_index) {
            const auto &prev_interval = intervals_.at(station_index).at(next_interval_index - 1);
            const auto &next_interval = intervals_.at(station_index).at(next_interval_index);
            const auto &switch_interval = intervals_.at(dummy_station_).at(next_interval_index);

            if (switch_interval.Begin < switch_interval.End) {
                CHECK_EQ(prev_interval.End, next_interval.Begin);
                mip_model_.addConstr(next_interval.Var <= prev_interval.Var + switch_interval.Var);
            }
        }
    }

    // set initial guess
    if (solution) {
        for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
            const auto station = model_->Station(station_index);
            auto &station_intervals = intervals_.at(station_index);
            const auto interval_end_it = std::end(station_intervals);

            std::unordered_set<BaseInterval> intervals_to_set;
            for (const auto &observation : solution->ObservationWindows(station)) {
                const auto start_time_index = model_->GetStartIndex(observation.begin());
                const auto end_time_index = model_->GetEndIndex(observation.end());

                for (auto interval_it = std::begin(station_intervals); interval_it != interval_end_it; ++interval_it) {
                    CHECK_EQ(interval_it->StationIndex, station_index);
                    if (interval_it->End <= start_time_index) { continue; }
                    if (interval_it->Begin >= end_time_index) { break; }

                    if (interval_it->End <= end_time_index) {
                        intervals_to_set.insert(*interval_it);
                    } else {
                        LOG(WARNING) << "Cannot initialize the interval " << *interval_it
                                     << " because it does not fully cover the time span ["
                                     << start_time_index << ", "
                                     << end_time_index << "]";
                    }
                }
            }

            for (auto interval_it = std::begin(station_intervals); interval_it != interval_end_it; ++interval_it) {
                const double is_set = intervals_to_set.find(*interval_it) != std::cend(intervals_to_set);
                interval_it->Var.set(GRB_DoubleAttr_Start, is_set);
            }
        }
    }
}

double quake::BaseIntervalMipModel::GetTrafficIndexUpperBound() const {
    double all_scenario_traffic_index = std::numeric_limits<double>::min();

    const auto num_scenarios = scenario_pool_.size();
    const auto start_time = model_->StartTime();
    const auto end_time = model_->EndTime();

    for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
        double scenario_traffic_index = std::numeric_limits<double>::max();
        for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
            const auto station = model_->Station(station_index);
            int64 keys_transferred = model_->WeatherAdjustedTransferredKeys(station,
                                                                            start_time,
                                                                            end_time,
                                                                            forecasts_.at(scenario_index));
            double station_traffic_index = keys_transferred / model_->TransferShare(station_index);
            scenario_traffic_index = std::min(scenario_traffic_index, station_traffic_index);
        }

        all_scenario_traffic_index = std::max(all_scenario_traffic_index, scenario_traffic_index);
    }

    return all_scenario_traffic_index;
}

std::unordered_map<quake::GroundStation,
        std::vector<boost::posix_time::time_period> > quake::BaseIntervalMipModel::GetObservations() const {
    std::vector<VarInterval> active_intervals;
    for (auto station_index = 0; station_index < num_stations_; ++station_index) {
        if (station_index == dummy_station_) { continue; }

        for (const auto &interval : intervals_.at(station_index)) {
            if (util::IsActive(interval.Var)) {
                active_intervals.push_back(interval);
            }
        }
    }

    std::sort(std::begin(active_intervals), std::end(active_intervals),
              [](const VarInterval &left, const VarInterval &right) -> bool {
                  return left.Begin <= right.Begin;
              });

    std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period>> observations;
    for (auto sequence_begin = 0; sequence_begin < active_intervals.size();) {
        auto sequence_end = sequence_begin + 1;
        for (; sequence_end < active_intervals.size()
               &&
               active_intervals.at(sequence_begin).StationIndex ==
               active_intervals.at(sequence_end).StationIndex;
               ++sequence_end);
        CHECK_LE(sequence_end, active_intervals.size());

        const auto &first_interval = active_intervals.at(sequence_begin);
        const auto station_index = first_interval.StationIndex;

        const auto interval_index
                = std::find(std::cbegin(intervals_.at(station_index)), std::cend(intervals_.at(station_index)),
                            first_interval) - std::cbegin(intervals_.at(station_index));

        const auto &switch_interval = intervals_.at(dummy_station_).at(interval_index);
        if (switch_interval.Begin < switch_interval.End) { CHECK(util::IsActive(switch_interval.Var)); }

        auto begin_index = switch_interval.Begin;
        const auto station = model_->Station(station_index);

        const auto is_first_interval = begin_index == 0;
        if ((is_first_interval && !active_intervals.empty() && active_intervals.front().Begin != 0)) {
            LOG(WARNING) << "Solution does not start with an active interval";
        }

        if (!is_first_interval && observations.empty()) {
            begin_index = 0;
        }

        auto prev_end_time = switch_interval.End;
        for (auto sequence_index = sequence_begin; sequence_index < sequence_end; ++sequence_index) {
            const auto &active_interval = active_intervals[sequence_index];

            CHECK_LE(prev_end_time, active_interval.Begin);
            if (prev_end_time < active_interval.Begin) {
                LOG(WARNING) << "Lack of continuity between " << model_->Time(prev_end_time)
                             << " and " << model_->Time(active_interval.Begin);
            }
            prev_end_time = active_interval.End;
        }

        auto end_index = active_intervals.at(sequence_end - 1).End;
        const auto is_last_interval = active_intervals.size() == sequence_end;
        if (is_last_interval && end_index != model_->TimeRange()) {
            LOG(WARNING) << "Solution does not end with an active interval";
            end_index = model_->TimeRange();
        }

        auto start_transfer_time = model_->Time(model_->GetStartTransferTimeIndex(station_index, begin_index));
        auto end_time = model_->Time(end_index);
        auto find_it = observations.find(station);
        boost::posix_time::time_period observation(start_transfer_time, end_time);
        if (find_it != std::end(observations)) {
            find_it->second.emplace_back(observation);
        } else {
            observations.emplace(station, std::vector<boost::posix_time::time_period>{observation});
        }
        sequence_begin = sequence_end;
    }

    return observations;
}

double quake::BaseIntervalMipModel::GetTrafficIndex(const quake::Solution &solution,
                                                    const quake::Forecast &forecast) const {
    auto traffic_index = std::numeric_limits<double>::max();

    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        const auto station = model_->Station(station_index);
        double keys_transferred = 0;
        for (const auto &window : solution.ObservationWindows(station)) {
            keys_transferred += model_->WeatherAdjustedTransferredKeys(station, window.begin(), window.end(), forecast);
        }

        const auto station_traffic_index = keys_transferred / model_->TransferShare(station_index);
        traffic_index = std::min(traffic_index, station_traffic_index);
    }

    return traffic_index;
}
