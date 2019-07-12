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


#include <ostream>
#include <glog/logging.h>
#include <nlohmann/json.hpp>

#include "solution.h"
#include "validator.h"

struct Observation {
    Observation(quake::GroundStation station, boost::posix_time::time_period period)
            : Station{std::move(station)},
              Period{period} {}

    quake::GroundStation Station;
    boost::posix_time::time_period Period;
};

void to_json(nlohmann::json &json, const Observation &observation) {
    nlohmann::json json_object;
    json_object["station"] = observation.Station;
    json_object["period"] = observation.Period;
    json = json_object;
}

std::ostream &operator<<(std::ostream &out, const Observation &observation) {
    nlohmann::json object = observation;
    out << object;
    return out;
}

void quake::Validator::Validate(const quake::Solution &solution) const {
    // final buffers are not negative
    for (const auto &station : solution.Stations()) {
        CHECK_GE(solution.FinalBuffer(station), 0.0);
    }

    // build a sorted vector of all observations to test for non-overlapping
    std::vector<Observation> observations;
    for (const auto &station : solution.Stations()) {
        for (const auto &period : solution.ObservationWindows(station)) {
            observations.emplace_back(station, period);
        }
    }

    if (observations.size() <= 1) {
        return;
    }

    std::sort(std::begin(observations), std::end(observations), [](const Observation &left, const Observation &right) -> bool {
        return left.Period.begin() < right.Period.begin();
    });

    // observation intervals are non-overlapping
    // every two regular observations are split by the idle observation of 30 seconds length
    auto prev_it = std::begin(observations);
    const auto end_it = std::end(observations);
    for (auto current_it = std::next(prev_it); current_it != end_it; ++current_it) {
        CHECK(!prev_it->Period.intersects(current_it->Period));

        if (prev_it->Station != current_it->Station) { // if station are different we expect one to be the dummy station
            const auto distance = current_it->Period.begin() - prev_it->Period.end();
            CHECK_GE(distance, problem_.SwitchDuration()) << "Observations of regular stations are not separated by the switch duration ("
                                                          << problem_.SwitchDuration() << "):" << *prev_it << " vs " << *current_it;
        } else if (prev_it->Station != GroundStation::None) { // if stations are the same we do not expect observation periods to be adjacent
            CHECK(!prev_it->Period.is_adjacent(current_it->Period));
        }
    }
}
