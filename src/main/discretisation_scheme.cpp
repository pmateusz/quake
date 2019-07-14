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

#include <utility>
#include <unordered_set>

#include "util/hash.h"

#include "discretisation_scheme.h"

quake::DiscretisationScheme quake::FixedDiscretisationSchemeFactory::Create(const ExtendedProblem &problem,
                                                                            const boost::posix_time::time_duration &time_step,
                                                                            const std::vector<GroundStation> &stations) const {
    std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period>> station_intervals;
    std::vector<boost::posix_time::time_period> switch_intervals;

    if (stations.empty()) {
        return {std::move(station_intervals), std::move(switch_intervals)};
    }

    // create station intervals
    for (const auto &station :stations) {
        std::vector<boost::posix_time::time_period> local_station_intervals;
        for (const auto &communication_window : problem.TransferWindows(station)) {
            const auto window_intervals = generate_observation_intervals(communication_window, time_step);
            std::copy(std::cbegin(window_intervals), std::cend(window_intervals), std::back_inserter(local_station_intervals));
        }
        station_intervals.emplace(station, std::move(local_station_intervals));
    }

    // obtain unique sorted vector of waiting intervals
    std::unordered_set<boost::posix_time::time_period> all_observation_intervals_set;
    for (const auto &element : station_intervals) {
        for (const auto &period : element.second) {
            all_observation_intervals_set.emplace(period);
        }
    }
    std::vector<boost::posix_time::time_period> all_observation_intervals;
    std::copy(std::cbegin(all_observation_intervals_set), std::cend(all_observation_intervals_set), std::back_inserter(all_observation_intervals));
    std::sort(std::begin(all_observation_intervals), std::end(all_observation_intervals),
              [](const boost::posix_time::time_period &left, const boost::posix_time::time_period &right) -> bool {
                  return left.begin() < right.begin();
              });

    // create switch intervals
    const auto observation_period_end = problem.ObservationPeriod().end();
    for (const auto &observation_interval : all_observation_intervals) {
        auto end_interval = observation_interval.begin();
        auto begin_interval = end_interval - problem.SwitchDuration();
        switch_intervals.emplace_back(begin_interval, end_interval);
    }

    return {std::move(station_intervals), std::move(switch_intervals)};
}

std::vector<boost::posix_time::time_period> quake::FixedDiscretisationSchemeFactory::generate_observation_intervals(
        const boost::posix_time::time_period &period, const boost::posix_time::time_duration &time_step) const {
    std::vector<boost::posix_time::time_period> result;
    result.reserve(period.length().total_seconds() / time_step.total_seconds() + 2);

    for (auto interval_begin = boost::posix_time::ptime(period.begin().date(), boost::posix_time::hours(period.begin().time_of_day().hours()));
         interval_begin < period.end();) {
        const auto interval_end = interval_begin + time_step;

        if (interval_end > period.begin()) {
            result.emplace_back(interval_begin, interval_end);
        }

        interval_begin = interval_end;
    }

    return result;
}


quake::DiscretisationScheme::DiscretisationScheme(
        std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period>> observation_intervals,
        std::vector<boost::posix_time::time_period> switch_intervals)
        : ObservationIntervals{std::move(observation_intervals)},
          SwitchIntervals{std::move(switch_intervals)} {}
