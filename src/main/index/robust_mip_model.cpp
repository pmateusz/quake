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

#include <boost/config.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include "robust_mip_model.h"
#include "util/hash.h"
#include "util/gurobi.h"

#include "robust/fixed_discretisation_scheme.h"
#include "robust/validator.h"

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
    try {
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
                keys_transferred += problem_->KeyRate(station, interval.Period(), ExtendedProblem::WeatherSample::None) * interval.Var();
            }

            mip_model_.addConstr(problem_->TransferShare(station) * traffic_index <= keys_transferred);
        }

        GRBLinExpr objective = traffic_index;
        mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
        mip_model_.setObjective(objective);

        if (gap_opt) {
            mip_model_.set(GRB_DoubleParam_MIPGap, *gap_opt);
        }

        mip_model_.optimize();

        using namespace util;

        const auto solver_status_code = mip_model_.get(GRB_IntAttr_Status);
        const auto solver_status = static_cast<SolverStatus>(solver_status_code);
        CHECK_NE(solver_status, SolverStatus::Loaded);
        CHECK_NE(solver_status, SolverStatus::InProgress);
        CHECK_NE(solver_status, SolverStatus::Numeric);
        CHECK_NE(solver_status, SolverStatus::Unbounded);
        CHECK_NE(solver_status, SolverStatus::InfiniteOrUnbounded);
        CHECK_NE(solver_status, SolverStatus::Infeasible);

        std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> > assignment;
        for (auto station_index = 0; station_index < num_stations; ++station_index) {
            const auto station = Station(station_index);
            std::vector<boost::posix_time::time_period> observations;

            const auto &station_intervals = intervals_.at(station_index);
            const auto num_intervals = station_intervals.size();
            for (auto interval_index = 0; interval_index < num_intervals;) {
                // find first active interval
                while (interval_index < num_intervals && !util::IsActive(station_intervals.at(interval_index).Var())) {
                    ++interval_index;
                }

                // check if active interval was found
                if (interval_index >= num_intervals) {
                    break;
                }

                auto observation = station_intervals.at(interval_index).Period();

                // extend interval if possible
                auto next_interval_index = interval_index + 1;
                while (next_interval_index < num_intervals) {
                    const auto &next_interval = station_intervals.at(next_interval_index);
                    if (next_interval.Period().is_adjacent(observation)) {
                        observation.merge(next_interval.Period());
                        ++next_interval_index;
                    } else {
                        break;
                    }
                }

                observations.emplace_back(observation);
                interval_index = next_interval_index;
            }

            assignment.emplace(station, std::move(observations));
        }

        // TODO: consider move to the problem?
        std::unordered_map<quake::GroundStation, int64> final_buffers;
        for (const auto &element: assignment) {
            const auto station = element.first;

            if (station == GroundStation::None) {
                final_buffers.emplace(station, 0);
                continue;
            }

            // number of keys transferred
            double keys_received = 0.0;
            for (const auto &observation_window : element.second) {
                keys_received += problem_->KeyRate(station, observation_window, ExtendedProblem::WeatherSample::Forecast);
            }

            // keys consumed
            const auto total_days = problem_->ObservationPeriod().length().total_seconds() / boost::posix_time::hours(24).total_seconds();
            const auto total_key_consumption = problem_->KeyConsumption(station) * total_days;

            // initial buffer
            const auto station_initial_buffer = problem_->InitialBuffer(station);

            CHECK_GE(station_initial_buffer - total_key_consumption, 0);

            final_buffers.emplace(station, station_initial_buffer + keys_received - total_key_consumption);
        }

        Solution solution{std::move(assignment), std::move(final_buffers)};

        robust::Validator validator{*problem_};
        validator.Validate(solution);

        return boost::make_optional(solution);
    } catch (const GRBException &exception) {
        LOG(FATAL) << "Gurobi exception: " << exception.getMessage() << " Error code: " << exception.getErrorCode();
    }
}

quake::robust::IntervalVar quake::RobustMipModel::CreateInterval(std::size_t station_index, boost::posix_time::time_period period) {
    std::stringstream label;
    label << "s" << station_index << "_" << period;
    return {station_index, period, mip_model_.addVar(0, 1, 0, GRB_BINARY, label.str())};
}
