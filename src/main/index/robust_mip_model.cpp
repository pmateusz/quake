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

#include <unordered_set>

#include "robust_mip_model.h"
#include "util/hash.h"

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
    const auto num_stations = stations_.size();
    intervals_.resize(num_stations);
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

    // constraint: at most one ground station is observed at a time

    // get all unique starting points
    std::unordered_set<boost::posix_time::ptime> point_time_set;
    for (auto station_index = 0; station_index < num_stations; ++station_index) {
        for (const auto &interval : intervals_.at(station_index)) {
            point_time_set.emplace(interval.Period().begin());
        }
    }
    std::vector<boost::posix_time::ptime> time_points;
    std::copy(std::begin(point_time_set), std::end(point_time_set), std::back_inserter(time_points));
    std::sort(std::begin(time_points), std::end(time_points));

    for (const auto &time_point : time_points) {
        GRBLinExpr point_cover = 0;

        for (auto station_index = 0; station_index < num_stations; ++station_index) {
            for (const auto &interval : intervals_.at(station_index)) {
                if (interval.Period().is_after(time_point)) {
                    break;
                }

                if (interval.Period().contains(time_point)) {
                    point_cover += interval.Var();
                }
            }
        }

        mip_model_.addConstr(point_cover <= 1);
    }

    // constraint: dummy station precedes observation of another ground station

    // build index of switch time intervals
    std::unordered_map<boost::posix_time::ptime, robust::IntervalVar> end_switch_interval;
    for (const auto &interval : intervals_.at(0)) {
        end_switch_interval.emplace(interval.Period().end(), interval);
    }

    for (auto station_index = 0; station_index < num_stations; ++station_index) {
        if (station_index == dummy_station_index) {
            continue;
        }

        const auto &station_intervals = intervals_.at(station_index);
        const auto interval_it_end = std::end(station_intervals);
        auto prev_interval_it = std::begin(station_intervals);
        if (prev_interval_it == interval_it_end) {
            continue;
        }

        for (auto interval_it = std::next(prev_interval_it); interval_it != interval_it_end; ++interval_it) {
            if (prev_interval_it->Period().end() == interval_it->Period().begin()) {
                mip_model_.addConstr(interval_it->Var() <= prev_interval_it->Var() + end_switch_interval.at(interval_it->Period().begin()).Var());
            }

            prev_interval_it = interval_it;
        }
    }

    // lambda
    GRBVar traffic_index = mip_model_.addVar(0, GRB_MAXINT, 0, GRB_CONTINUOUS, "traffic_index");

    // constraint: traffic index
    for (auto station_index = 0; station_index < num_stations; ++station_index) {
        if (station_index == dummy_station_index) {
            continue;
        }

        const auto station = Station(station_index);
        const auto &station_intervals = intervals_.at(station_index);
        GRBLinExpr keys_transferred = 0;
        for (const auto &interval : station_intervals) {
            keys_transferred += problem_->KeyRate(station, interval.Period()) * interval.Var();
        }

        mip_model_.addConstr(problem_->TransferShare(station) * traffic_index <= keys_transferred);
    }

    GRBLinExpr objective = traffic_index;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    mip_model_.setObjective(objective);

    mip_model_.optimize();

    // TODO: build assignment
    // TODO: validate assignment

    return boost::none;
}

quake::robust::IntervalVar quake::RobustMipModel::CreateInterval(std::size_t station_index, boost::posix_time::time_period period) {
    std::stringstream label;
    label << "s" << station_index << "_" << period;
    return {station_index, period, mip_model_.addVar(0, 1, 0, GRB_BINARY, label.str())};
}
