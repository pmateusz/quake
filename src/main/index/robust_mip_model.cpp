#include <utility>

//
// Copyright 2019 Mateusz Polnik
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

#include "robust_mip_model.h"

#include "robust/fixed_discretisation_scheme.h"

quake::RobustMipModel::RobustMipModel(const quake::ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : problem_{problem},
          interval_step_{std::move(interval_step)},
          stations_{GroundStation::None},
          mip_environment_{},
          mip_model_{mip_environment_} {
    std::copy(std::cbegin(problem_->GroundStations()), std::cend(problem_->GroundStations()), std::back_inserter(stations_));
    for (std::size_t index_pos = 0; index_pos < stations_.size(); ++index_pos) {
        station_indices_.emplace(stations_.at(index_pos), index_pos);
    }
}

boost::optional<quake::Solution> quake::RobustMipModel::Solve(const boost::optional<boost::posix_time::time_duration> &time_limit_opt,
                                                              const boost::optional<double> &gap_opt) {
    robust::FixedDiscretisationScheme discretisation_scheme{*problem_, interval_step_};
    const auto scheme = discretisation_scheme.Build();

    // create intervals for regular stations
    intervals_.resize(stations_.size());
    for (const auto &observation_element : scheme.ObservationIntervals) {
        const auto station_index = Index(observation_element.first);
        for (const auto &period : observation_element.second) {
            intervals_.at(station_index).emplace_back(CreateInterval(station_index, period));
        }
    }
    CHECK(intervals_.front().empty());

    // create intervals for dummy station
    const auto dummy_station_index = Index(GroundStation::None);
    for (const auto &period : scheme.SwitchIntervals) {
        intervals_.at(dummy_station_index).emplace_back(CreateInterval(dummy_station_index, period));
    }

    // get indices of all valid starting points

    // at most one ground station is observed at a time

    // dummy station precedes observation of another ground station

    return boost::none;
}

quake::robust::IntervalVar quake::RobustMipModel::CreateInterval(std::size_t station_index, boost::posix_time::time_period period) {
    std::stringstream label;
    label << "s" << station_index << "_" << period;
    return {station_index, period, mip_model_.addVar(0, 1, 0, GRB_BINARY, label.str())};
}
