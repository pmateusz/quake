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

#include <utility>
#include <unordered_set>

#include <glog/logging.h>

#include "util/math.h"
#include "base_interval_mip_model.h"
#include "discretisation_scheme.h"

quake::BaseIntervalMipModel::BaseIntervalMipModel(const ExtendedProblem *problem,
                                                  boost::posix_time::time_duration interval_step,
                                                  std::vector<Forecast> forecasts)
        : BaseMipModel{problem},
          interval_step_{std::move(interval_step)},
          forecasts_{std::move(forecasts)} {
    for (const auto &station : BaseMipModel::Stations()) {
        if (station != GroundStation::None) {
            observable_stations_.emplace_back(station);
        }
    }
}


void quake::BaseIntervalMipModel::Build(const boost::optional<Solution> &solution) {
    CHECK(intervals_.empty());

    // variables: create intervals
    FixedDiscretisationSchemeFactory scheme;
    const auto intervals = scheme.Create(*problem_, interval_step_, ObservableStations());

    // create intervals for regular stations
    const auto num_stations = Stations().size();
    intervals_.resize(num_stations);
    for (const auto &observation_element : intervals.ObservationIntervals) {
        const auto station_index = Index(observation_element.first);
        for (const auto &period : observation_element.second) {
            intervals_.at(station_index).emplace_back(CreateInterval(station_index, period));
        }
    }
    CHECK(intervals_.front().empty());

    // create intervals for dummy station
    for (const auto &period : intervals.SwitchIntervals) {
        intervals_.at(dummy_station_index_).emplace_back(CreateInterval(dummy_station_index_, period));
    }

    scenario_pool_ = ScenarioPool(intervals.ObservationIntervals, forecasts_, problem_);

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
    std::unordered_map<boost::posix_time::ptime, IntervalVar> end_switch_interval;
    for (const auto &interval : intervals_.at(dummy_station_index_)) {
        end_switch_interval.emplace(interval.Period().end(), interval);
    }

    for (auto station_index = 0; station_index < num_stations; ++station_index) {
        if (station_index == dummy_station_index_) {
            continue;
        }

        const auto &station_intervals = intervals_.at(station_index);
        const auto interval_it_end = std::end(station_intervals);
        auto prev_interval_it = std::begin(station_intervals);
        if (prev_interval_it == interval_it_end) {
            continue;
        }

        // set switch for the first interval
        mip_model_.addConstr(prev_interval_it->Var() <= end_switch_interval.at(prev_interval_it->Period().begin()).Var());

        // set switch for the second interval and others
        for (auto interval_it = std::next(prev_interval_it); interval_it != interval_it_end; ++interval_it) {
            const auto &switch_interval = end_switch_interval.at(interval_it->Period().begin());
            if (prev_interval_it->Period().end() == interval_it->Period().begin()) {
                mip_model_.addConstr(interval_it->Var() <= prev_interval_it->Var() + switch_interval.Var());
            } else {
                mip_model_.addConstr(interval_it->Var() <= switch_interval.Var());
            }

            prev_interval_it = interval_it;
        }
    }

    // set initial guess
    if (solution) {
        LOG(FATAL) << "Warm start not implemented";
//        for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
//            const auto station = model_->Station(station_index);
//            auto &station_intervals = intervals_.at(station_index);
//            const auto interval_end_it = std::end(station_intervals);
//
//            std::unordered_set<BaseInterval> intervals_to_set;
//            for (const auto &observation : solution->ObservationWindows(station)) {
//                const auto start_time_index = model_->GetStartIndex(observation.begin());
//                const auto end_time_index = model_->GetEndIndex(observation.end());
//
//                for (auto interval_it = std::begin(station_intervals); interval_it != interval_end_it; ++interval_it) {
//                    CHECK_EQ(interval_it->StationIndex, station_index);
//                    if (interval_it->End <= start_time_index) { continue; }
//                    if (interval_it->Begin >= end_time_index) { break; }
//
//                    if (interval_it->End <= end_time_index) {
//                        intervals_to_set.insert(*interval_it);
//                    } else {
//                        LOG(WARNING) << "Cannot initialize the interval " << *interval_it
//                                     << " because it does not fully cover the time span ["
//                                     << start_time_index << ", "
//                                     << end_time_index << "]";
//                    }
//                }
//            }
//
//            for (auto interval_it = std::begin(station_intervals); interval_it != interval_end_it; ++interval_it) {
//                const double is_set = intervals_to_set.find(*interval_it) != std::cend(intervals_to_set);
//                interval_it->Var.set(GRB_DoubleAttr_Start, is_set);
//            }
//        }
    }
}

std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> > quake::BaseIntervalMipModel::GetObservations() const {

    const auto &forecast = problem_->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast);

    std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> > assignment;
    for (const auto &station : ObservableStations()) {
        const auto &station_intervals = StationIntervals(station);

        double sub_interval_keys_received = 0;
        for (const auto &interval : station_intervals) {
            if (util::IsActive(interval.Var())) {
                sub_interval_keys_received += problem_->KeyRate(station, interval.Period(), forecast);
            }
        }

        std::vector<boost::posix_time::time_period> observations;
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
                if (!util::IsActive(next_interval.Var()) || !observation.is_adjacent(next_interval.Period())) {
                    break;
                }

                observation = observation.span(next_interval.Period());
                ++next_interval_index;
            }

            observations.emplace_back(observation);
            interval_index = next_interval_index;
        }

        // self-test: ensure all intervals are included in collapsed intervals
        for (const auto &interval : station_intervals) {
            if (!util::IsActive(interval.Var())) {
                continue;
            }

            bool found_aggregate = false;
            for (const auto &aggregated_interval : observations) {
                found_aggregate = aggregated_interval.contains(interval.Period());
                if (found_aggregate) {
                    break;
                }
            }
            CHECK(found_aggregate);
        }

        double observations_keys_received = 0;
        for (const auto &observation: observations) {
            observations_keys_received += problem_->KeyRate(station, observation, forecast);
        }
        util::is_nearly_eq(sub_interval_keys_received, observations_keys_received);

        assignment.emplace(station, std::move(observations));
    }

    return assignment;
}

double quake::BaseIntervalMipModel::GetTrafficIndexUpperBound() const {
    double traffic_index = std::numeric_limits<double>::max();

    const auto max_observation_period = problem_->ObservationPeriod();
    for (const auto &station : Stations()) {
        if (station == GroundStation::None) {
            continue;
        }

        const auto total_keys_transferred = problem_->KeyRate(station, max_observation_period);
        const auto station_traffic_index = total_keys_transferred / problem_->TransferShare(station);
        traffic_index = std::min(station_traffic_index, traffic_index);
    }

    return traffic_index;
}

quake::IntervalVar quake::BaseIntervalMipModel::CreateInterval(std::size_t station_index, const boost::posix_time::time_period &period) {
    std::stringstream label;
    label << "s" << station_index << "_" << period;
    return {station_index, period, mip_model_.addVar(0, 1, 0, GRB_BINARY, label.str())};
}

void quake::BaseIntervalMipModel::AppendMetadata(quake::Metadata &metadata) {
    BaseMipModel::AppendMetadata(metadata);

    metadata.SetProperty(Metadata::Property::IntervalStep, interval_step_);
}

std::vector<quake::IntervalVar> quake::BaseIntervalMipModel::GetIntervals(const quake::GroundStation &station,
                                                                          const boost::posix_time::time_period &time_period) const {
    const auto &station_intervals = intervals_.at(Index(station));

    std::vector<IntervalVar> result;
    for (const auto &interval : station_intervals) {
        const auto &interval_period = interval.Period();
        if (time_period.contains(interval_period)) {
            result.emplace_back(interval);
            continue;
        }

        if (time_period.is_before(interval_period.begin())) {
            break;
        }

        CHECK(!time_period.intersects(interval_period))
                        << "Interval " << interval_period
                        << " intersects with " << time_period << " but is not fully contained";
    }
    return result;
}
